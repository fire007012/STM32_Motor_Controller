#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"

#define MOTOR_COUNT                         4U
#define MOTOR_CMD_QUEUE_LENGTH              16U

typedef struct {
    int32_t target_velocity;
    int32_t current_velocity;
    int32_t target_position;
    int32_t position_feedback;
    uint16_t status_word;
    uint16_t fault_code;
    uint8_t fault_flag;
} Motor_State_t;

typedef struct {
    uint32_t rx_count;
    uint32_t enqueue_drop_count;
    uint32_t exec_ok_count;
    uint32_t exec_fail_count;
    uint32_t timeout_drop_count;
    uint16_t last_seq;
    uint8_t last_cmd;
    uint8_t last_result;
} Motor_CommStats_t;

typedef enum {
    MOTOR_REPORT_BASIC = 0x01U,
    MOTOR_REPORT_POSITION = 0x02U,
    MOTOR_REPORT_VELOCITY = 0x04U,
    MOTOR_REPORT_TARGET = 0x08U
} Motor_ReportMask_t;

typedef enum {
    MOTOR_CMD_NONE = 0x00,
    MOTOR_CMD_SET_VELOCITY = 0x01,
    MOTOR_CMD_SET_POSITION = 0x02,
    MOTOR_CMD_ESTOP = 0x03,
    MOTOR_CMD_SET_PROFILE = 0x04,
    MOTOR_CMD_SYNC_START = 0x05,
    MOTOR_CMD_SET_RESPONSE_POLICY = 0x06,
    MOTOR_CMD_SET_REPORT_MASK = 0x07
} Motor_CommandCode_t;

typedef struct {
    uint8_t cmd;
    uint8_t motor_idx;
    int32_t value;
    uint8_t param0;
    uint8_t param1;
    uint16_t seq;
} Motor_Command_t;

extern Motor_State_t motors[MOTOR_COUNT];

void motor_control_init(void);
uint8_t motor_enqueue_command_from_isr(const Motor_Command_t *cmd);
uint8_t motor_fetch_command(Motor_Command_t *cmd, uint32_t timeout_ms);
uint16_t motor_allocate_cmd_seq(void);
void motor_record_rx_result(uint8_t enqueue_ok);

void motor_set_velocity(uint8_t idx, int32_t vel);
void motor_set_velocity_ex(uint8_t idx, int32_t vel, uint8_t accel_level, uint8_t sync_flag);
void motor_set_position(uint8_t idx, int32_t pos);
void motor_set_position_ex(uint8_t idx, int32_t pos, uint8_t accel_level, uint8_t mode, uint8_t sync_flag);
void motor_set_speed_profile(uint8_t idx, uint16_t speed_rpm, uint8_t accel_level);
void motor_trigger_sync_motion(void);
HAL_StatusTypeDef motor_send_multi_cmd(const uint8_t *cmd_stream, uint16_t stream_len);
void motor_set_response_policy(uint8_t wait_ack);
void motor_stop_all(void);
void motor_update_status(void);

HAL_StatusTypeDef motor_apply_command(const Motor_Command_t *cmd);
void motor_set_ros_alive(void);
uint8_t motor_is_ros_timeout(uint32_t timeout_ms);
void motor_set_report_mask(uint8_t report_mask);
uint8_t motor_get_report_mask(void);
void motor_get_comm_stats(Motor_CommStats_t *stats_out);
void motor_set_timeout_drop_count(uint32_t timeout_drop_count);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
