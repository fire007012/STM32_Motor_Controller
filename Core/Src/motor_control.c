#include "motor_control.h"

#include "modbus_can.h"

Motor_State_t motors[MOTOR_COUNT] = {0};

static osMessageQueueId_t motor_cmd_queue = NULL;
static uint32_t ros_last_heartbeat_tick = 0U;
static uint16_t motor_speed_profile[MOTOR_COUNT] = {1000U, 1000U, 1000U, 1000U};
static uint8_t motor_accel_profile[MOTOR_COUNT] = {10U, 10U, 10U, 10U};
static uint8_t motor_report_mask = (uint8_t)(MOTOR_REPORT_BASIC | MOTOR_REPORT_POSITION);
static uint16_t motor_seq_counter = 0U;
static Motor_CommStats_t motor_comm_stats = {0};

void motor_control_init(void)
{
    uint8_t i;

    if (motor_cmd_queue == NULL) {
        motor_cmd_queue = osMessageQueueNew(MOTOR_CMD_QUEUE_LENGTH, sizeof(Motor_Command_t), NULL);
    }
    ros_last_heartbeat_tick = HAL_GetTick();

    for (i = 0U; i < MOTOR_COUNT; i++) {
        (void)zdt_motor_enable((uint8_t)(i + 1U), 1U, ZDT_SYNC_IMMEDIATE);
        HAL_Delay(2U);
    }
}

uint8_t motor_enqueue_command_from_isr(const Motor_Command_t *cmd)
{
    osStatus_t status;

    if ((cmd == NULL) || (motor_cmd_queue == NULL)) {
        return 0U;
    }

    status = osMessageQueuePut(motor_cmd_queue, cmd, 0U, 0U);
    return (status == osOK) ? 1U : 0U;
}

uint8_t motor_fetch_command(Motor_Command_t *cmd, uint32_t timeout_ms)
{
    osStatus_t status;

    if ((cmd == NULL) || (motor_cmd_queue == NULL)) {
        return 0U;
    }

    status = osMessageQueueGet(motor_cmd_queue, cmd, NULL, timeout_ms);
    return (status == osOK) ? 1U : 0U;
}

uint16_t motor_allocate_cmd_seq(void)
{
    motor_seq_counter++;
    return motor_seq_counter;
}

void motor_record_rx_result(uint8_t enqueue_ok)
{
    motor_comm_stats.rx_count++;
    if (enqueue_ok == 0U) {
        motor_comm_stats.enqueue_drop_count++;
    }
}

void motor_set_velocity(uint8_t idx, int32_t vel)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].target_velocity = vel;
    (void)zdt_motor_set_speed((uint8_t)(idx + 1U), vel, motor_accel_profile[idx], ZDT_SYNC_IMMEDIATE);
}

void motor_set_velocity_ex(uint8_t idx, int32_t vel, uint8_t accel_level, uint8_t sync_flag)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].target_velocity = vel;
    motor_accel_profile[idx] = accel_level;
    (void)zdt_motor_set_speed((uint8_t)(idx + 1U), vel, accel_level, (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE);
}

void motor_set_position(uint8_t idx, int32_t pos)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].target_position = pos;
    (void)zdt_motor_set_position((uint8_t)(idx + 1U),
                                 pos,
                                 motor_speed_profile[idx],
                                 motor_accel_profile[idx],
                                 0U,
                                 ZDT_SYNC_IMMEDIATE);
}

void motor_set_position_ex(uint8_t idx, int32_t pos, uint8_t accel_level, uint8_t mode, uint8_t sync_flag)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].target_position = pos;
    motor_accel_profile[idx] = accel_level;
    (void)zdt_motor_set_position((uint8_t)(idx + 1U),
                                 pos,
                                 motor_speed_profile[idx],
                                 accel_level,
                                 mode,
                                 (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE);
}

void motor_set_speed_profile(uint8_t idx, uint16_t speed_rpm, uint8_t accel_level)
{
    if (idx >= MOTOR_COUNT) {
        return;
    }

    if (speed_rpm > 3000U) {
        speed_rpm = 3000U;
    }

    motor_speed_profile[idx] = speed_rpm;
    motor_accel_profile[idx] = accel_level;
}

void motor_trigger_sync_motion(void)
{
    (void)zdt_trigger_sync_start();
}

HAL_StatusTypeDef motor_send_multi_cmd(const uint8_t *cmd_stream, uint16_t stream_len)
{
    return zdt_send_multi_motor_command(cmd_stream, stream_len);
}

void motor_set_response_policy(uint8_t wait_ack)
{
    if (wait_ack != 0U) {
        zdt_set_response_policy(ZDT_RESPONSE_WAIT_ACK);
    } else {
        zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    }
}

void motor_stop_all(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motors[i].target_velocity = 0;
        (void)zdt_motor_stop((uint8_t)(i + 1U), ZDT_SYNC_IMMEDIATE);
    }
}

static void motor_status_read_cb(uint8_t slave, uint8_t status_flags)
{
    uint8_t idx;

    if ((slave < 1U) || (slave > MOTOR_COUNT)) {
        return;
    }

    idx = (uint8_t)(slave - 1U);
    motors[idx].status_word = status_flags;
    motors[idx].fault_code = status_flags;
    motors[idx].fault_flag = ((status_flags & 0x0CU) != 0U) ? 1U : 0U;
}

static void motor_position_read_cb(uint8_t slave, int32_t pos)
{
    uint8_t idx;

    if ((slave < 1U) || (slave > MOTOR_COUNT)) {
        return;
    }

    idx = (uint8_t)(slave - 1U);
    motors[idx].position_feedback = pos;
}

static void motor_velocity_read_cb(uint8_t slave, int32_t vel)
{
    uint8_t idx;

    if ((slave < 1U) || (slave > MOTOR_COUNT)) {
        return;
    }

    idx = (uint8_t)(slave - 1U);
    motors[idx].current_velocity = vel;
}

void motor_update_status(void)
{
    static uint8_t i = 0U;
    static uint8_t phase = 0U;

    if (phase == 0U) {
        if (zdt_read_motor_status((uint8_t)(i + 1U), motor_status_read_cb) == HAL_OK) {
            phase = 1U;
        }
    } else if (phase == 1U) {
        if (zdt_read_realtime_position((uint8_t)(i + 1U), motor_position_read_cb) == HAL_OK) {
            phase = 2U;
        }
    } else {
        if (zdt_read_realtime_speed((uint8_t)(i + 1U), motor_velocity_read_cb) == HAL_OK) {
            phase = 0U;
            i = (uint8_t)((i + 1U) % MOTOR_COUNT);
        }
    }
}

HAL_StatusTypeDef motor_apply_command(const Motor_Command_t *cmd)
{
    uint8_t i;
    HAL_StatusTypeDef status = HAL_OK;

    if (cmd == NULL) {
        return HAL_ERROR;
    }

    motor_comm_stats.last_seq = cmd->seq;
    motor_comm_stats.last_cmd = cmd->cmd;

    switch (cmd->cmd) {
        case MOTOR_CMD_SET_VELOCITY:
            if (cmd->motor_idx == 0xFFU) {
                for (i = 0U; i < MOTOR_COUNT; i++) {
                    motor_set_velocity_ex(i, cmd->value, cmd->param0, cmd->param1);
                }
            } else {
                motor_set_velocity_ex(cmd->motor_idx, cmd->value, cmd->param0, cmd->param1);
            }
            break;

        case MOTOR_CMD_ESTOP:
            motor_stop_all();
            break;

        case MOTOR_CMD_SET_POSITION:
            if (cmd->motor_idx == 0xFFU) {
                for (i = 0U; i < MOTOR_COUNT; i++) {
                    motor_set_position_ex(i,
                                          cmd->value,
                                          cmd->param0,
                                          (uint8_t)(cmd->param1 & 0x03U),
                                          (uint8_t)((cmd->param1 >> 2) & 0x01U));
                }
            } else {
                motor_set_position_ex(cmd->motor_idx,
                                      cmd->value,
                                      cmd->param0,
                                      (uint8_t)(cmd->param1 & 0x03U),
                                      (uint8_t)((cmd->param1 >> 2) & 0x01U));
            }
            break;

        case MOTOR_CMD_SET_PROFILE:
            if (cmd->motor_idx == 0xFFU) {
                for (i = 0U; i < MOTOR_COUNT; i++) {
                    motor_set_speed_profile(i, (uint16_t)cmd->value, cmd->param0);
                }
            } else {
                motor_set_speed_profile(cmd->motor_idx, (uint16_t)cmd->value, cmd->param0);
            }
            break;

        case MOTOR_CMD_SYNC_START:
            motor_trigger_sync_motion();
            break;

        case MOTOR_CMD_SET_RESPONSE_POLICY:
            motor_set_response_policy(cmd->param0);
            break;

        case MOTOR_CMD_SET_REPORT_MASK:
            motor_set_report_mask((uint8_t)cmd->value);
            break;

        default:
            status = HAL_ERROR;
            break;
    }

    if (status == HAL_OK) {
        motor_comm_stats.exec_ok_count++;
        motor_comm_stats.last_result = 0U;
    } else {
        motor_comm_stats.exec_fail_count++;
        motor_comm_stats.last_result = 1U;
    }

    return status;
}

void motor_set_ros_alive(void)
{
    ros_last_heartbeat_tick = HAL_GetTick();
}

uint8_t motor_is_ros_timeout(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    return ((now - ros_last_heartbeat_tick) > timeout_ms) ? 1U : 0U;
}

void motor_set_report_mask(uint8_t report_mask)
{
    uint8_t valid_mask = (uint8_t)(MOTOR_REPORT_BASIC | MOTOR_REPORT_POSITION | MOTOR_REPORT_VELOCITY | MOTOR_REPORT_TARGET);

    if ((report_mask & valid_mask) == 0U) {
        motor_report_mask = (uint8_t)(MOTOR_REPORT_BASIC | MOTOR_REPORT_POSITION);
    } else {
        motor_report_mask = (uint8_t)(report_mask & valid_mask);
    }
}

uint8_t motor_get_report_mask(void)
{
    return motor_report_mask;
}

void motor_get_comm_stats(Motor_CommStats_t *stats_out)
{
    if (stats_out == NULL) {
        return;
    }

    *stats_out = motor_comm_stats;
}

void motor_set_timeout_drop_count(uint32_t timeout_drop_count)
{
    motor_comm_stats.timeout_drop_count = timeout_drop_count;
}
