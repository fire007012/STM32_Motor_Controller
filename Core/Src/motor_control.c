#include "motor_control.h"

#include "zdt_can_driver.h"
#include "zdt_status.h"
#include "chassis_safety_config.h"
#include <string.h>

Motor_State_t motors[MOTOR_COUNT] = {0};

static osMessageQueueId_t motor_cmd_queue = NULL;
static Motor_Command_t urgent_stop;
static uint8_t urgent_stop_pending = 0U;
static Motor_Command_t local_sync_start;
static uint8_t local_sync_start_pending = 0U;
static uint32_t ros_last_heartbeat_tick = 0U;
static uint8_t ros_heartbeat_seen = 0U;
static uint8_t motor_slave_addr[MOTOR_COUNT] = {1U, 2U, 3U, 4U};
static uint16_t motor_speed_profile[MOTOR_COUNT] = {1000U, 1000U, 1000U, 1000U};
static uint8_t motor_accel_profile[MOTOR_COUNT] = {10U, 10U, 10U, 10U};
static uint8_t motor_report_mask = MOTOR_REPORT_DEFAULT;
static uint16_t motor_seq_counter = 0U;
static Motor_CommStats_t motor_comm_stats = {0};

static uint8_t staged_mask, local_stop_pending, safety_latched;
static uint8_t status_poll_index, status_poll_type, speed_poll_index, poll_speed_next;

/* One command at a time, correlated through native confirmation. Navigation
 * and braking decisions belong to ROS; sync=0 never waits for other wheels. */
static struct {
    Motor_Command_t cmd;
    HAL_StatusTypeDef result;
    uint32_t tag;
    uint8_t remaining, reason;
    uint8_t native, slave, function;
    uint8_t count, submitted;
} execution;
static Motor_Command_t deferred_command;
static uint8_t deferred_pending;
static uint32_t execution_tag;
static const Motor_Command_t *diagnostic_command;
static uint8_t failure_reason;
volatile Motor_CommandDiagnostic_t motor_command_diagnostics[MOTOR_DIAGNOSTIC_LENGTH];
volatile Motor_CommandDiagnostic_t motor_last_failure;
volatile uint32_t motor_command_diagnostic_count;

static void diagnose(const Motor_Command_t *cmd, uint8_t phase, HAL_StatusTypeDef status,
                     uint8_t reason, uint8_t slave, uint8_t function, uint8_t native)
{
    if (cmd == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Motor_CommandDiagnostic_t d = {.tick=HAL_GetTick(),
        .age_ms=HAL_GetTick()-cmd->received_tick, .seq=cmd->seq, .cmd=cmd->cmd,
        .motor_idx=cmd->motor_idx, .phase=phase, .hal_status=(uint8_t)status,
        .reason=reason, .slave=slave, .function=function, .native_result=native,
        .staged_mask=staged_mask, .collision_mask=0U};
    motor_command_diagnostics[motor_command_diagnostic_count++ % MOTOR_DIAGNOSTIC_LENGTH] = d;
    if (phase == MOTOR_PHASE_FINAL && status != HAL_OK &&
        (reason != MOTOR_REASON_CANCELED || motor_last_failure.reason == MOTOR_REASON_NONE)) motor_last_failure = d;
    if (primask == 0U) __enable_irq();
}

static void driver_result(uint32_t tag, uint8_t slave, uint8_t function,
                          HAL_StatusTypeDef status, uint8_t native, uint8_t canceled)
{
    if (execution.count && execution.tag == tag && execution.remaining) {
        /* Stops issued during rejection must not confirm the rejected motion. */
        uint8_t expected = execution.cmd.cmd == MOTOR_CMD_SET_VELOCITY ? ZDT_CMD_SPEED_MODE :
            execution.cmd.cmd == MOTOR_CMD_SET_POSITION ? ZDT_CMD_POSITION_MODE :
            execution.cmd.cmd == MOTOR_CMD_SYNC_START ? ZDT_CMD_SYNC_TRIGGER : ZDT_CMD_EMERGENCY_STOP;
        if (function != expected) return;
        if (status != HAL_OK && execution.result == HAL_OK) {
            execution.result = status;
            execution.reason = native ? MOTOR_REASON_NATIVE_REJECT :
                status == HAL_TIMEOUT ? MOTOR_REASON_RESPONSE_TIMEOUT :
                canceled ? MOTOR_REASON_CANCELED : MOTOR_REASON_TRANSMIT;
        }
        if (execution.result == HAL_OK || execution.function == 0U) {
            execution.native = native;
            execution.slave = slave;
            execution.function = function;
        } else if (native) {
            execution.native = native;
            execution.slave = slave;
            execution.function = function;
        }
        execution.remaining--;
        diagnose(&execution.cmd, MOTOR_PHASE_DRIVER, status, execution.reason, slave, function, native);
    }
}

static uint8_t motion_allowed(void)
{
    if (local_stop_pending) { failure_reason=MOTOR_REASON_STOP_PENDING; return 0U; }
    if (motor_is_ros_timeout(CHASSIS_HEARTBEAT_TIMEOUT_MS)) {
        failure_reason=MOTOR_REASON_HEARTBEAT;
        return 0U;
    }
    for (uint8_t i=0U; i<MOTOR_COUNT; ++i) {
        if (motors[i].fault_flag) { failure_reason=MOTOR_REASON_FAULT; return 0U; }
    }
    return 1U;
}

void motor_safety_poll(void)
{
    /* The motor task owns all autonomous parking. Stop once per new link/fault
     * condition, and retry only a stop that could not be submitted. */
    uint8_t conditions = motor_is_ros_timeout(CHASSIS_HEARTBEAT_TIMEOUT_MS) ? 1U : 0U;
    for (uint8_t i=0U; i<MOTOR_COUNT; ++i)
        if (motors[i].fault_flag) conditions |= 2U;
    uint8_t new_condition = conditions & (uint8_t)~safety_latched;
    safety_latched = conditions;
    if (new_condition || local_stop_pending) (void)motor_stop_all();
}

static HAL_StatusTypeDef submit_motion(uint8_t idx, int32_t value, uint8_t accel,
                                       uint8_t mode, uint8_t sync, uint8_t command)
{
    if ((idx >= MOTOR_COUNT && idx != 0xFFU) ||
        (command == MOTOR_CMD_SET_POSITION && mode > 2U)) {
        failure_reason = MOTOR_REASON_ARGUMENT;
        return HAL_ERROR;
    }
    if (!motion_allowed()) return HAL_ERROR;
    if (sync && staged_mask == (1U << MOTOR_COUNT)-1U) staged_mask = 0U;
    if (!sync) staged_mask = 0U; /* a later start must not revive old cache */
    for (uint8_t i=0U; i<MOTOR_COUNT; ++i) {
        if (idx != 0xFFU && idx != i) continue;
        if (!motion_allowed()) return HAL_ERROR;
        HAL_StatusTypeDef status = command == MOTOR_CMD_SET_VELOCITY ?
            zdt_motor_set_speed(motor_slave_addr[i], value, accel, sync) :
            zdt_motor_set_position(motor_slave_addr[i], value, motor_speed_profile[i], accel, mode, sync);
        if (status != HAL_OK) { failure_reason=MOTOR_REASON_SUBMIT; return status; }
        motor_accel_profile[i] = accel;
        if (command == MOTOR_CMD_SET_VELOCITY) {
            int32_t clamped = value > 3000 ? 3000 : value < -3000 ? -3000 : value;
            motors[i].target_velocity = clamped * 10;
        } else motors[i].target_position = value;
        if (sync) staged_mask |= (uint8_t)(1U << i);
    }
    return HAL_OK;
}

void motor_control_init(void)
{
    motor_report_mask = MOTOR_REPORT_DEFAULT;
    if (motor_cmd_queue == NULL) {
        motor_cmd_queue = osMessageQueueNew(MOTOR_CMD_QUEUE_LENGTH, sizeof(Motor_Command_t), NULL);
    }
    if (motor_cmd_queue == NULL) {
        Error_Handler();
    }
    ros_last_heartbeat_tick = 0U;
    ros_heartbeat_seen = 0U;
    urgent_stop_pending = 0U;
    local_sync_start_pending = 0U;
    staged_mask = local_stop_pending = safety_latched = 0U;
    status_poll_index = status_poll_type = speed_poll_index = 0U;
    poll_speed_next = 1U;
    memset(&execution, 0, sizeof(execution));
    deferred_pending = 0U;
    execution_tag = 0U;
    diagnostic_command = NULL;
    motor_command_diagnostic_count = 0U;
    motor_last_failure = (Motor_CommandDiagnostic_t){0};
    zdt_status_init();
}

HAL_StatusTypeDef motor_enable_all(void)
{
    uint8_t i;
    HAL_StatusTypeDef status = HAL_OK;
    for (i = 0U; i < MOTOR_COUNT; i++) {
        HAL_StatusTypeDef result = zdt_motor_enable(motor_slave_addr[i], 1U, ZDT_SYNC_IMMEDIATE);
        if (result != HAL_OK) { status = result; }
    }
    return status;
}

HAL_StatusTypeDef motor_configure_addresses(const uint8_t *addr_list, uint8_t count)
{
    uint8_t i;
    uint8_t j;

    if ((addr_list == NULL) || (count < MOTOR_COUNT)) {
        return HAL_ERROR;
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (addr_list[i] == 0U) {
            return HAL_ERROR;
        }

        for (j = (uint8_t)(i + 1U); j < MOTOR_COUNT; j++) {
            if (addr_list[i] == addr_list[j]) {
                return HAL_ERROR;
            }
        }
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_slave_addr[i] = addr_list[i];
    }
    zdt_set_sync_response_slave(addr_list[0]);
    if (primask == 0U) { __enable_irq(); }
    return HAL_OK;
}

uint8_t motor_get_address(uint8_t idx)
{
    if (idx >= MOTOR_COUNT) {
        return 0U;
    }

    return motor_slave_addr[idx];
}

static uint8_t motor_find_index_by_slave(uint8_t slave)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (motor_slave_addr[i] == slave) {
            return i;
        }
    }

    return MOTOR_COUNT;
}

uint8_t motor_enqueue_command_from_isr(const Motor_Command_t *cmd)
{
    osStatus_t status;

    if ((cmd == NULL) || (motor_cmd_queue == NULL)) {
        return 0U;
    }

    /* Reserve an independent slot: a motion burst must never reject ESTOP. */
    if (cmd->cmd == MOTOR_CMD_ESTOP) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (urgent_stop_pending != 0U) {
            if (primask == 0U) { __enable_irq(); }
            return 0U; /* an earlier stop is already pending */
        }
        urgent_stop = *cmd;
        urgent_stop_pending = 1U;
        if (primask == 0U) { __enable_irq(); }
        return 1U;
    }

    Motor_Command_t stamped = *cmd;
    stamped.received_tick = HAL_GetTick();
    status = osMessageQueuePut(motor_cmd_queue, &stamped, 0U, 0U);
    diagnose(&stamped, MOTOR_PHASE_RECEIVED, status == osOK ? HAL_OK : HAL_BUSY,
             status == osOK ? MOTOR_REASON_NONE : MOTOR_REASON_SUBMIT, 0U, 0U, 0U);
    return (status == osOK) ? 1U : 0U;
}

uint8_t motor_request_local_sync_start(uint16_t seq)
{
    uint32_t primask = __get_PRIMASK();

    if (motor_is_ros_ready(CHASSIS_HEARTBEAT_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    __disable_irq();
    if (local_sync_start_pending != 0U) {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    local_sync_start.cmd = MOTOR_CMD_SYNC_START;
    local_sync_start.motor_idx = 0xFFU;
    local_sync_start.value = 0;
    local_sync_start.param0 = 0U;
    local_sync_start.param1 = 0U;
    local_sync_start.seq = seq;
    local_sync_start.source = MOTOR_CMD_SOURCE_LOCAL_BUTTON;
    local_sync_start.received_tick = HAL_GetTick();
    local_sync_start_pending = 1U;

    if (primask == 0U) { __enable_irq(); }
    return 1U;
}

uint8_t motor_fetch_command(Motor_Command_t *cmd, uint32_t timeout_ms)
{
    osStatus_t status;

    if ((cmd == NULL) || (motor_cmd_queue == NULL)) {
        return 0U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (urgent_stop_pending != 0U) {
        *cmd = urgent_stop;
        urgent_stop_pending = 0U;
        if (primask == 0U) { __enable_irq(); }
        return 1U;
    }
    if (primask == 0U) { __enable_irq(); }

    status = osMessageQueueGet(motor_cmd_queue, cmd, NULL, timeout_ms);
    return (status == osOK) ? 1U : 0U;
}

uint16_t motor_allocate_cmd_seq(void)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t seq;

    __disable_irq();
    motor_seq_counter++;
    if (motor_seq_counter == 0U) { motor_seq_counter++; }
    seq = motor_seq_counter;
    if (primask == 0U) { __enable_irq(); }
    return seq;
}

static uint8_t fetch_ordered_command(Motor_Command_t *cmd)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t fetched;
    if (deferred_pending && !urgent_stop_pending) {
        *cmd = deferred_command;
        deferred_pending = 0U;
        fetched = 1U;
    } else fetched = motor_fetch_command(cmd, 0U);
    if (!fetched && local_sync_start_pending) {
        *cmd = local_sync_start;
        local_sync_start_pending = 0U;
        fetched = 1U;
    }
    if (primask == 0U) __enable_irq();
    return fetched;
}

static void submit_execution(void)
{
    Motor_Command_t *cmd = &execution.cmd;
    failure_reason = MOTOR_REASON_NONE;
    execution.submitted = 1U;
    if ((cmd->cmd == MOTOR_CMD_SET_VELOCITY || cmd->cmd == MOTOR_CMD_SET_POSITION ||
         cmd->cmd == MOTOR_CMD_SYNC_START) &&
        HAL_GetTick()-cmd->received_tick > CHASSIS_COMMAND_TIMEOUT_MS) {
        execution.result = HAL_TIMEOUT;
        execution.reason = MOTOR_REASON_QUEUE_EXPIRED;
        diagnose(cmd, MOTOR_PHASE_VALIDATE, HAL_TIMEOUT, MOTOR_REASON_QUEUE_EXPIRED, 0U, 0U, 0U);
        return; /* stale input is rejected, never used as a parking decision */
    }
    uint8_t tracked = cmd->cmd == MOTOR_CMD_SET_VELOCITY || cmd->cmd == MOTOR_CMD_SET_POSITION ||
                      cmd->cmd == MOTOR_CMD_SYNC_START || cmd->cmd == MOTOR_CMD_ESTOP;
    if (++execution_tag == 0U) ++execution_tag;
    execution.tag = execution_tag;
    execution.remaining = tracked ?
        ((cmd->motor_idx == 0xFFU && (cmd->cmd == MOTOR_CMD_SET_VELOCITY || cmd->cmd == MOTOR_CMD_SET_POSITION)) ? MOTOR_COUNT : 1U) : 0U;
    diagnostic_command = cmd;
    zdt_set_command_context(execution.tag, tracked ? driver_result : NULL);
    HAL_StatusTypeDef status = motor_apply_command(cmd);
    zdt_set_command_context(0U, NULL);
    diagnostic_command = NULL;
    if (status != HAL_OK) {
        execution.remaining = 0U;
        execution.result = status;
        execution.reason = failure_reason ? failure_reason : MOTOR_REASON_ARGUMENT;
    }
}

uint8_t motor_process_next_command(Motor_Command_t *cmd, HAL_StatusTypeDef *result)
{
    if (cmd == NULL || result == NULL) return 0U;
    /* ESTOP can interrupt an in-flight native transaction. */
    if (urgent_stop_pending && execution.count) {
        Motor_Command_t stop = urgent_stop;
        (void)motor_stop_all();
        deferred_command = stop;
        deferred_pending = 1U;
    }
    if (execution.submitted) return 0U;
    if (!fetch_ordered_command(cmd)) return 0U;
    execution.count = 1U;
    execution.cmd = *cmd;
    *result = HAL_BUSY; /* submitted/pending; take_result owns the terminal ACK */
    diagnose(cmd, MOTOR_PHASE_COLLECT, HAL_BUSY, MOTOR_REASON_NONE, 0U, 0U, 0U);
    submit_execution();
    return 1U;
}

uint8_t motor_take_command_result(Motor_Command_t *cmd, HAL_StatusTypeDef *result)
{
    if (!execution.submitted || cmd == NULL || result == NULL) return 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (execution.remaining && execution.cmd.cmd != MOTOR_CMD_ESTOP &&
        HAL_GetTick()-execution.cmd.received_tick > CHASSIS_COMMAND_TIMEOUT_MS) {
        execution.result = HAL_TIMEOUT;
        execution.reason = MOTOR_REASON_RESPONSE_TIMEOUT;
    }
    uint8_t reason = execution.reason;
    if (execution.result != HAL_OK &&
        (reason == MOTOR_REASON_SUBMIT || reason == MOTOR_REASON_NATIVE_REJECT ||
         reason == MOTOR_REASON_RESPONSE_TIMEOUT || reason == MOTOR_REASON_TRANSMIT)) {
        /* A partially executed/unknown motion is a low-level safety failure.
         * Validation rejects alone never broadcast a stop. */
        (void)motor_stop_all();
        execution.remaining = 0U;
    }
    if (execution.remaining) {
        if (primask == 0U) __enable_irq();
        return 0U;
    }
    *cmd = execution.cmd;
    *result = execution.result;
    motor_comm_stats.last_seq = cmd->seq;
    motor_comm_stats.last_cmd = cmd->cmd;
    motor_comm_stats.last_result = *result == HAL_OK ? 0U : 1U;
    if (*result == HAL_OK) motor_comm_stats.exec_ok_count++;
    else motor_comm_stats.exec_fail_count++;
    diagnose(cmd, MOTOR_PHASE_FINAL, *result, execution.reason,
             execution.slave, execution.function, execution.native);
    memset(&execution, 0, sizeof(execution));
    if (primask == 0U) __enable_irq();
    return 1U;
}

void motor_record_rx_result(uint8_t enqueue_ok)
{
    motor_comm_stats.rx_count++;
    if (enqueue_ok == 0U) {
        motor_comm_stats.enqueue_drop_count++;
    }
}

HAL_StatusTypeDef motor_set_velocity(uint8_t idx, int32_t vel)
{
    if (idx >= MOTOR_COUNT) { return HAL_ERROR; }
    return motor_set_velocity_ex(idx, vel, motor_accel_profile[idx], ZDT_SYNC_IMMEDIATE);
}

HAL_StatusTypeDef motor_set_velocity_ex(uint8_t idx, int32_t vel, uint8_t accel_level, uint8_t sync_flag)
{
    return submit_motion(idx, vel, accel_level, 0U, sync_flag, MOTOR_CMD_SET_VELOCITY);
}

HAL_StatusTypeDef motor_set_position(uint8_t idx, int32_t pos)
{
    if (idx >= MOTOR_COUNT) { return HAL_ERROR; }
    return motor_set_position_ex(idx, pos, motor_accel_profile[idx], 0U, ZDT_SYNC_IMMEDIATE);
}

HAL_StatusTypeDef motor_set_position_ex(uint8_t idx, int32_t pos, uint8_t accel_level, uint8_t mode, uint8_t sync_flag)
{
    return submit_motion(idx, pos, accel_level, mode, sync_flag, MOTOR_CMD_SET_POSITION);
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

HAL_StatusTypeDef motor_trigger_sync_motion(void)
{
    if (!motion_allowed()) return HAL_ERROR;
    if (staged_mask != (1U << MOTOR_COUNT)-1U) {
        failure_reason = MOTOR_REASON_CACHE_INCOMPLETE;
        return HAL_ERROR; /* report only; ROS decides whether to stop */
    }
    HAL_StatusTypeDef status = zdt_trigger_sync_start();
    if (status == HAL_OK) staged_mask = 0U;
    else failure_reason = MOTOR_REASON_SUBMIT;
    return status;
}

HAL_StatusTypeDef motor_send_multi_cmd(const uint8_t *cmd_stream, uint16_t stream_len)
{
    /* Opaque AA has no host-command/native-result correlation. Use 0x100. */
    (void)cmd_stream; (void)stream_len;
    return HAL_ERROR;
}

void motor_set_response_policy(uint8_t wait_ack)
{
    if (wait_ack != 0U) {
        zdt_set_response_policy(ZDT_RESPONSE_WAIT_ACK);
    } else {
        zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    }
}

HAL_StatusTypeDef motor_stop_all(void)
{
    uint8_t i;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    staged_mask = 0U;
    deferred_pending = 0U;
    /* Remove older motion and reads before enqueuing stop commands. */
    zdt_can_driver_cancel();
    urgent_stop_pending = 0U;
    local_sync_start_pending = 0U;
    if (motor_cmd_queue != NULL) {
        (void)osMessageQueueReset(motor_cmd_queue);
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motors[i].target_velocity = 0;
    }
    /* A broadcast stop needs no ACK and cannot be held behind a missing motor. */
    status = zdt_motor_stop(0U, ZDT_SYNC_IMMEDIATE);
    local_stop_pending = status != HAL_OK;
    if (status != HAL_OK) { motor_record_stop_failure(); }
    if (primask == 0U) { __enable_irq(); }
    return status;
}

static void motor_status_read_cb(uint8_t slave, uint8_t status_flags)
{
    uint8_t idx;

    idx = motor_find_index_by_slave(slave);
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].status_word = status_flags;
    motors[idx].fault_code = status_flags;
    motors[idx].fault_flag = zdt_status_is_fault(status_flags);
    zdt_status_update_status(idx, status_flags);
}

static void motor_position_read_cb(uint8_t slave, int32_t pos)
{
    uint8_t idx;

    idx = motor_find_index_by_slave(slave);
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].position_feedback = pos;
}

static void motor_velocity_read_cb(uint8_t slave, int32_t vel)
{
    uint8_t idx;

    idx = motor_find_index_by_slave(slave);
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].current_velocity = vel;
    zdt_status_update_speed(idx, vel);
}

void motor_update_status(uint8_t max_requests)
{
    uint8_t issued = 0U;
    HAL_StatusTypeDef status;

    if (max_requests == 0U) {
        return;
    }

    while ((issued < max_requests) && (zdt_can_driver_is_busy() == 0U)) {
        /* Preserve feedback cadence. ROS evaluates freshness and body motion;
         * a background read timeout alone does not park the chassis. */
        if (poll_speed_next) status = zdt_read_realtime_speed(motor_slave_addr[speed_poll_index], motor_velocity_read_cb);
        else if (status_poll_type == 0U) status = zdt_read_motor_status(motor_slave_addr[status_poll_index], motor_status_read_cb);
        else status = zdt_read_realtime_position(motor_slave_addr[status_poll_index], motor_position_read_cb);
        if (status != HAL_OK) { break; }
        issued++;
        if (poll_speed_next) speed_poll_index = (uint8_t)((speed_poll_index + 1U) % MOTOR_COUNT);
        else if (++status_poll_type >= 2U) {
            status_poll_type = 0U;
            status_poll_index = (uint8_t)((status_poll_index + 1U) % MOTOR_COUNT);
        }
        poll_speed_next = !poll_speed_next;
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

    if (((cmd->cmd == MOTOR_CMD_SET_VELOCITY) || (cmd->cmd == MOTOR_CMD_SET_POSITION) ||
         (cmd->cmd == MOTOR_CMD_SET_PROFILE)) &&
        (cmd->motor_idx >= MOTOR_COUNT) && (cmd->motor_idx != 0xFFU)) {
        status = HAL_ERROR;
        goto record_result;
    }

    switch (cmd->cmd) {
        case MOTOR_CMD_SET_VELOCITY:
            status = submit_motion(cmd->motor_idx, cmd->value, cmd->param0, 0U, cmd->param1, cmd->cmd);
            break;

        case MOTOR_CMD_ESTOP:
            status = motor_stop_all();
            break;

        case MOTOR_CMD_SET_POSITION:
            status = submit_motion(cmd->motor_idx, cmd->value, cmd->param0,
                                   (uint8_t)(cmd->param1 & 0x03U),
                                   (uint8_t)((cmd->param1 >> 2) & 1U), cmd->cmd);
            break;

        case MOTOR_CMD_SET_PROFILE:
            if ((cmd->value < 0) || (cmd->value > 3000)) { status = HAL_ERROR; break; }
            if (cmd->motor_idx == 0xFFU) {
                for (i = 0U; i < MOTOR_COUNT; i++) {
                    motor_set_speed_profile(i, (uint16_t)cmd->value, cmd->param0);
                }
            } else {
                motor_set_speed_profile(cmd->motor_idx, (uint16_t)cmd->value, cmd->param0);
            }
            break;

        case MOTOR_CMD_SYNC_START:
            status = motor_trigger_sync_motion();
            break;

        case MOTOR_CMD_SET_RESPONSE_POLICY:
            motor_set_response_policy(cmd->param0);
            break;

        case MOTOR_CMD_SET_REPORT_MASK:
            motor_set_report_mask((uint8_t)cmd->value);
            break;

        case MOTOR_CMD_SET_ADDRESS_MAP: {
            uint8_t addr_map[MOTOR_COUNT];
            uint32_t raw;

            raw = (uint32_t)cmd->value;
            addr_map[0] = (uint8_t)(raw & 0xFFU);
            addr_map[1] = (uint8_t)((raw >> 8) & 0xFFU);
            addr_map[2] = (uint8_t)((raw >> 16) & 0xFFU);
            addr_map[3] = (uint8_t)((raw >> 24) & 0xFFU);
            status = motor_configure_addresses(addr_map, MOTOR_COUNT);
            break;
        }

        case MOTOR_CMD_HEARTBEAT:
            break;

        default:
            status = HAL_ERROR;
            break;
    }

record_result:
    diagnose(diagnostic_command, MOTOR_PHASE_SUBMIT, status, failure_reason, 0U, 0U, 0U);

    return status;
}

void motor_set_ros_alive(void)
{
    ros_last_heartbeat_tick = HAL_GetTick();
    ros_heartbeat_seen = 1U;
}

void motor_set_ros_startup_grace(void)
{
    ros_last_heartbeat_tick = HAL_GetTick();
}

uint8_t motor_is_ros_timeout(uint32_t timeout_ms)
{
    uint32_t now = HAL_GetTick();
    return ((now - ros_last_heartbeat_tick) > timeout_ms) ? 1U : 0U;
}

uint8_t motor_is_ros_ready(uint32_t timeout_ms)
{
    if (ros_heartbeat_seen == 0U) {
        return 0U;
    }
    return (motor_is_ros_timeout(timeout_ms) == 0U) ? 1U : 0U;
}

void motor_set_report_mask(uint8_t report_mask)
{
    uint8_t valid_mask = (uint8_t)(MOTOR_REPORT_BASIC | MOTOR_REPORT_POSITION | MOTOR_REPORT_VELOCITY | MOTOR_REPORT_TARGET);

    if ((report_mask & valid_mask) == 0U) {
        motor_report_mask = MOTOR_REPORT_DEFAULT;
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

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *stats_out = motor_comm_stats;
    if (primask == 0U) { __enable_irq(); }
}

void motor_set_timeout_drop_count(uint32_t timeout_drop_count)
{
    motor_comm_stats.timeout_drop_count = timeout_drop_count;
}

void motor_set_can2_tx_fail_count(uint32_t can2_tx_fail_count)
{
    motor_comm_stats.can2_tx_fail_count = can2_tx_fail_count;
}

void motor_record_can1_tx_failure(void)
{
    motor_comm_stats.can1_tx_fail_count++;
}

void motor_record_stop_failure(void)
{
    motor_comm_stats.stop_fail_count++;
}
