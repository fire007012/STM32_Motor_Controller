#ifndef __ZDT_CAN_DRIVER_H
#define __ZDT_CAN_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Y42 CAN protocol uses extended ID: (Addr << 8) | PacketIndex */
#define ZDT_CAN_PACKET_SHIFT                 8U
#define ZDT_CAN_PACKET_MASK                  0xFFU
#define ZDT_CAN_CHECK_BYTE                   0x6BU

#define ZDT_CMD_MOTOR_ENABLE                 0xF3U
#define ZDT_CMD_SPEED_MODE                   0xF6U
#define ZDT_CMD_POSITION_MODE                0xFDU
#define ZDT_CMD_EMERGENCY_STOP               0xFEU
#define ZDT_CMD_SYNC_TRIGGER                 0xFFU

#define ZDT_CMD_READ_REALTIME_SPEED          0x35U
#define ZDT_CMD_READ_REALTIME_POSITION       0x36U
#define ZDT_CMD_READ_MOTOR_STATUS            0x3AU

#define ZDT_SYNC_IMMEDIATE                   0x00U
#define ZDT_SYNC_CACHE                       0x01U

typedef void (*zdt_speed_cb_t)(uint8_t slave, int32_t speed_rpm);
typedef void (*zdt_position_cb_t)(uint8_t slave, int32_t position_raw);
typedef void (*zdt_status_cb_t)(uint8_t slave, uint8_t status_flags);

typedef enum {
	ZDT_RESPONSE_WAIT_ACK = 0,
	ZDT_RESPONSE_FIRE_AND_FORGET = 1
} zdt_response_policy_t;

void zdt_can_driver_init(CAN_HandleTypeDef *hcan_bus);

HAL_StatusTypeDef zdt_motor_enable(uint8_t slave, uint8_t enable, uint8_t sync_flag);
HAL_StatusTypeDef zdt_motor_set_speed(uint8_t slave, int32_t speed_rpm, uint8_t accel_level, uint8_t sync_flag);
HAL_StatusTypeDef zdt_motor_set_position(uint8_t slave,
										 int32_t pulses,
										 uint16_t speed_rpm,
										 uint8_t accel_level,
										 uint8_t mode,
										 uint8_t sync_flag);
HAL_StatusTypeDef zdt_motor_stop(uint8_t slave, uint8_t sync_flag);
HAL_StatusTypeDef zdt_trigger_sync_start(void);
HAL_StatusTypeDef zdt_send_multi_motor_command(const uint8_t *cmd_stream, uint16_t stream_len);

void zdt_set_response_policy(zdt_response_policy_t policy);
zdt_response_policy_t zdt_get_response_policy(void);

HAL_StatusTypeDef zdt_read_realtime_speed(uint8_t slave, zdt_speed_cb_t callback);
HAL_StatusTypeDef zdt_read_realtime_position(uint8_t slave, zdt_position_cb_t callback);
HAL_StatusTypeDef zdt_read_motor_status(uint8_t slave, zdt_status_cb_t callback);

void zdt_can_driver_process_response(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8]);
void zdt_can_driver_timeout_poll(void);
uint32_t zdt_can_driver_get_timeout_drop_count(void);
uint32_t zdt_can_driver_get_tx_fail_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __ZDT_CAN_DRIVER_H */
