#ifndef __ZDT_STATUS_H
#define __ZDT_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* From status command 0x3A bit field */
#define ZDT_STATUS_BIT_STALL_FLAG           0x04U
#define ZDT_STATUS_BIT_STALL_PROTECT        0x08U
#define ZDT_STATUS_FAULT_MASK               0xFCU

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

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_STATUS_H */
