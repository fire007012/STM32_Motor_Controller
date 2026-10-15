#ifndef CHASSIS_COLLISION_H
#define CHASSIS_COLLISION_H
#include "distance_sensor.h"
#include "chassis_safety_config.h"

typedef struct { float vx, vy, wz; } chassis_velocity_t;
void chassis_collision_init(void);
void chassis_collision_update_range(const distance_sensor_sample_t *sample);
void chassis_velocity_from_rpm(const float rpm[4], chassis_velocity_t *body);
float chassis_collision_stop_mm(float approach_mps);
/* Returns a front/left/right/rear bitmask. Call with the unmodified request. */
uint8_t chassis_collision_check(const chassis_velocity_t *requested,
                                const chassis_velocity_t *measured, uint32_t now);
#endif
