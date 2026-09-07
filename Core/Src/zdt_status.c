#include "zdt_status.h"

#include "motor_control.h"

static uint8_t status_snapshot[MOTOR_COUNT] = {0};
static int32_t speed_snapshot[MOTOR_COUNT] = {0};
static uint8_t estop_pending = 0U;
static zdt_estop_event_t estop_event = {0};

void zdt_status_init(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        status_snapshot[i] = 0U;
        speed_snapshot[i] = 0;
    }

    estop_pending = 0U;
    estop_event.motor_idx = 0U;
    estop_event.status_flags = 0U;
    estop_event.speed_rpm = 0;
}

void zdt_status_update_status(uint8_t motor_idx, uint8_t status_flags)
{
    if (motor_idx >= MOTOR_COUNT) {
        return;
    }

    status_snapshot[motor_idx] = status_flags;

    if (zdt_status_classify(status_flags) == ZDT_EVENT_CLASS_FAULT) {
        estop_event.motor_idx = motor_idx;
        estop_event.status_flags = status_flags;
        estop_event.speed_rpm = speed_snapshot[motor_idx];
        estop_pending = 1U;
    }
}

void zdt_status_update_speed(uint8_t motor_idx, int32_t speed_rpm)
{
    if (motor_idx >= MOTOR_COUNT) {
        return;
    }

    speed_snapshot[motor_idx] = speed_rpm;
}

uint8_t zdt_status_take_estop_event(zdt_estop_event_t *event_out)
{
    if ((event_out == NULL) || (estop_pending == 0U)) {
        return 0U;
    }

    *event_out = estop_event;
    estop_pending = 0U;
    return 1U;
}

uint8_t zdt_status_is_fault(uint8_t status_flags)
{
    return ((status_flags & ZDT_STATUS_FAULT_MASK) != 0U) ? 1U : 0U;
}

zdt_event_class_t zdt_status_classify(uint8_t status_flags)
{
    if ((status_flags & ZDT_STATUS_FAULT_MASK) != 0U) {
        return ZDT_EVENT_CLASS_FAULT;
    }

    if ((status_flags & ZDT_STATUS_WARNING_MASK) != 0U) {
        return ZDT_EVENT_CLASS_WARNING;
    }

    if ((status_flags & ZDT_STATUS_INFO_MASK) != 0U) {
        return ZDT_EVENT_CLASS_INFO;
    }

    return ZDT_EVENT_CLASS_NONE;
}
