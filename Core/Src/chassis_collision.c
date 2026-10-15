#include "chassis_collision.h"
#include <string.h>

static distance_sensor_sample_t ranges[4];
static uint8_t blocked[4], previous_mode[4];
static float held_threshold[4];

void chassis_collision_init(void)
{
    memset(ranges, 0, sizeof(ranges));
    memset(blocked, 0, sizeof(blocked));
    memset(previous_mode, 0, sizeof(previous_mode));
    memset(held_threshold, 0, sizeof(held_threshold));
}

void chassis_collision_update_range(const distance_sensor_sample_t *sample)
{
    uint8_t direction;
    if (sample == NULL) return;
    if (sample->sensor_id <= DIST_SENSOR_RIGHT) direction = sample->sensor_id;
    else if (sample->sensor_id == DIST_SENSOR_REAR) direction = 3U;
    else return; /* IDs 3/4 never enter chassis protection. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ranges[direction] = *sample;
    if (primask == 0U) __enable_irq();
}

void chassis_velocity_from_rpm(const float rpm[4], chassis_velocity_t *body)
{
    static const float matrix[3][4] = CHASSIS_RPM_TO_BODY_MATRIX;
    float q[3] = {0};
    for (uint8_t axis = 0U; axis < 3U; ++axis)
        for (uint8_t wheel = 0U; wheel < 4U; ++wheel)
            q[axis] += matrix[axis][wheel] * rpm[wheel];
    body->vx = q[0]; body->vy = q[1]; body->wz = q[2];
}

float chassis_collision_stop_mm(float v)
{
    float mm = 1000.0f * (v*v/(2.0f*CHASSIS_BRAKE_DECEL_MPS2) +
                          v*CHASSIS_REACTION_TIME_S) + CHASSIS_EDGE_MARGIN_MM;
    return mm > CHASSIS_MIN_STOP_MM ? mm : CHASSIS_MIN_STOP_MM;
}

uint8_t chassis_collision_check(const chassis_velocity_t *request,
                                const chassis_velocity_t *measured, uint32_t now)
{
    static const uint16_t inset[4] = CHASSIS_SENSOR_INSETS_MM;
    float req[4] = {request->vx, request->vy, -request->vy, -request->vx};
    float obs[4] = {measured->vx, measured->vy, -measured->vy, -measured->vx};
    uint8_t turning = (request->wz > CHASSIS_SPEED_EPSILON ||
                       request->wz < -CHASSIS_SPEED_EPSILON ||
                       measured->wz > CHASSIS_SPEED_EPSILON ||
                       measured->wz < -CHASSIS_SPEED_EPSILON);
    uint8_t result = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        distance_sensor_sample_t sample = ranges[i]; /* caller holds motor lock */
        float v = req[i] > obs[i] ? req[i] : obs[i];
        uint8_t mode = v > CHASSIS_SPEED_EPSILON ? 2U : turning ? 1U : 0U;
        if (mode == 0U || !sample.valid ||
            (sample.status & DISTANCE_SENSOR_STATUS_VALID) == 0U ||
            (sample.status & (DISTANCE_SENSOR_STATUS_OUT_OF_RANGE |
              DISTANCE_SENSOR_STATUS_TIMEOUT | DISTANCE_SENSOR_STATUS_I2C_ERROR |
              DISTANCE_SENSOR_STATUS_LOW_QUALITY)) != 0U ||
            sample.distance_mm < CHASSIS_RANGE_MIN_MM ||
            sample.distance_mm > CHASSIS_RANGE_MAX_MM ||
            (uint32_t)(now - sample.timestamp_ms) > CHASSIS_RANGE_FRESH_MS) {
            blocked[i] = previous_mode[i] = 0U;
            continue;
        }
        /* Validate raw distance BEFORE subtracting. Zero is a valid clearance. */
        uint16_t clearance = sample.distance_mm > inset[i] ? (uint16_t)(sample.distance_mm - inset[i]) : (uint16_t)0;
        float threshold = mode == 2U ? chassis_collision_stop_mm(v) : CHASSIS_ROTATION_STOP_MM;
        if (previous_mode[i] != mode) blocked[i] = 0U;
        previous_mode[i] = mode;
        if (blocked[i]) {
            /* Braking feedback must not shrink the release boundary. */
            if (threshold > held_threshold[i]) held_threshold[i] = threshold;
            if ((float)clearance > held_threshold[i] + CHASSIS_RELEASE_MM) blocked[i] = 0U;
        } else if ((float)clearance <= threshold) {
            blocked[i] = 1U;
            held_threshold[i] = threshold;
        }
        if (blocked[i]) result |= (uint8_t)(1U << i);
    }
    return result;
}
