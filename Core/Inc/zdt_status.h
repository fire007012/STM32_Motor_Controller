#ifndef __ZDT_STATUS_H
#define __ZDT_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* From status command 0x3A bit field (vendor doc section 3.4.14) */
#define ZDT_STATUS_BIT_ENABLE_STATE         0x01U
#define ZDT_STATUS_BIT_POSITION_REACHED     0x02U
#define ZDT_STATUS_BIT_STALL_FLAG           0x04U
#define ZDT_STATUS_BIT_STALL_PROTECT        0x08U
#define ZDT_STATUS_BIT_LEFT_LIMIT           0x10U
#define ZDT_STATUS_BIT_RIGHT_LIMIT          0x20U
#define ZDT_STATUS_BIT_POWER_LOSS           0x80U

/* Hard faults that require immediate stop/event escalation. */
#define ZDT_STATUS_FAULT_MASK               (ZDT_STATUS_BIT_STALL_PROTECT | ZDT_STATUS_BIT_POWER_LOSS)

/* Warning bits that should be observable but do not mandate estop by themselves. */
#define ZDT_STATUS_WARNING_MASK             (ZDT_STATUS_BIT_STALL_FLAG)

/* Informational/runtime state bits. */
#define ZDT_STATUS_INFO_MASK                (ZDT_STATUS_BIT_ENABLE_STATE | \
                                            ZDT_STATUS_BIT_POSITION_REACHED | \
                                            ZDT_STATUS_BIT_LEFT_LIMIT | \
                                            ZDT_STATUS_BIT_RIGHT_LIMIT)

typedef enum {
    ZDT_EVENT_CLASS_NONE = 0U,
    ZDT_EVENT_CLASS_INFO = 1U,
    ZDT_EVENT_CLASS_WARNING = 2U,
    ZDT_EVENT_CLASS_FAULT = 3U
} zdt_event_class_t;

typedef struct {
    uint8_t motor_idx;
    uint8_t status_flags;
    int32_t speed_rpm;
} zdt_estop_event_t;

void zdt_status_init(void);
void zdt_status_update_status(uint8_t motor_idx, uint8_t status_flags);
void zdt_status_update_speed(uint8_t motor_idx, int32_t speed_rpm);
uint8_t zdt_status_take_estop_event(zdt_estop_event_t *event_out);
uint8_t zdt_status_is_fault(uint8_t status_flags);
zdt_event_class_t zdt_status_classify(uint8_t status_flags);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_STATUS_H */
