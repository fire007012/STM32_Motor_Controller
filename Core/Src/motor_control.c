#include "motor_control.h"

#include "zdt_can_driver.h"
#include "zdt_status.h"
#include "chassis_collision.h"
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
static uint8_t motor_report_mask = (uint8_t)(MOTOR_REPORT_BASIC | MOTOR_REPORT_POSITION);
static uint16_t motor_seq_counter = 0U;
static Motor_CommStats_t motor_comm_stats = {0};

typedef struct {
    int32_t value;
    uint32_t tick;
    uint16_t speed_rpm;
    uint8_t command, mode;
} motion_slot_t;
static motion_slot_t staged[MOTOR_COUNT];
static uint8_t staged_mask;
static float requested_rpm[MOTOR_COUNT];
static uint8_t request_active, physical_estop, local_stop_pending;
static uint32_t request_tick;
static uint32_t active_wheel_tick[MOTOR_COUNT];
static uint32_t velocity_tick[MOTOR_COUNT], position_tick[MOTOR_COUNT];
static uint8_t velocity_seen[MOTOR_COUNT], position_seen[MOTOR_COUNT];
static int64_t last_position_target[MOTOR_COUNT];
static uint8_t position_target_seen[MOTOR_COUNT];
static uint8_t status_poll_index, status_poll_type, speed_poll_index, poll_speed_next;

static uint8_t independent_stop(void)
{
    if (physical_estop || motor_is_ros_timeout(CHASSIS_HEARTBEAT_TIMEOUT_MS)) return 1U;
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
        if (motors[i].fault_flag) return 1U;
    return 0U;
}

static uint8_t collision_blocks(const float rpm[MOTOR_COUNT])
{
    chassis_velocity_t request, measured = {0};
    float feedback[MOTOR_COUNT];
    uint8_t fresh = 1U, moving = 0U;
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        if (rpm[i] != 0.0f) moving = 1U;
        if (!velocity_seen[i] || now - velocity_tick[i] > CHASSIS_FEEDBACK_FRESH_MS) fresh = 0U;
        feedback[i] = (float)motors[i].current_velocity * 0.1f; /* wire = 0.1 RPM */
    }
    if (local_stop_pending || independent_stop() || (moving && !CHASSIS_KINEMATICS_READY)) return 1U;
    chassis_velocity_from_rpm(rpm, &request);
    if (fresh) chassis_velocity_from_rpm(feedback, &measured);
    return chassis_collision_check(&request, &measured, now) != 0U;
}

/* This stop is releasable: cancel the interrupted request, never emit an
 * emergency event or set sensor bit 0x20, and require a NEW valid request.
 * Keep the host queue so a newer safe lateral request can still execute. */
static HAL_StatusTypeDef collision_stop(void)
{
    zdt_can_driver_cancel();
    staged_mask = request_active = 0U;
    memset(requested_rpm, 0, sizeof(requested_rpm));
    memset(position_target_seen, 0, sizeof(position_target_seen));
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) motors[i].target_velocity = 0;
    HAL_StatusTypeDef status = zdt_motor_stop(0U, ZDT_SYNC_IMMEDIATE);
    local_stop_pending = status != HAL_OK;
    if (status != HAL_OK) motor_record_stop_failure();
    return status;
}

void motor_update_distance(const distance_sensor_sample_t *sample)
{
    chassis_collision_update_range(sample);
}

void motor_set_physical_estop(uint8_t pressed)
{
    physical_estop = pressed != 0U;
}

void motor_safety_poll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t expired = HAL_GetTick() - request_tick > CHASSIS_COMMAND_TIMEOUT_MS;
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i)
        if (requested_rpm[i] != 0.0f && HAL_GetTick() - active_wheel_tick[i] > CHASSIS_COMMAND_TIMEOUT_MS)
            expired = 1U;
    if (request_active && (expired || independent_stop())) {
        (void)motor_stop_all();
    } else if (local_stop_pending || (request_active && collision_blocks(requested_rpm))) {
        (void)collision_stop();
    }
    if (primask == 0U) __enable_irq();
}

static uint8_t slot_to_speed(uint8_t idx, const motion_slot_t *slot, float *rpm, int64_t *target)
{
    if (slot->command == MOTOR_CMD_SET_VELOCITY) {
        int32_t value = slot->value;
        if (value > 3000) value = 3000;
        if (value < -3000) value = -3000;
        *rpm = (float)value;
        return 1U;
    }
    /* Position protocol is unchanged (pulses -> 0.1 degree by 9/8).
     * Absolute and previous-target moves need a fresh position reference;
     * guessing the sign from the command value is unsafe. */
    int64_t delta = ((int64_t)slot->value * 9) / 8;
    uint8_t fresh = position_seen[idx] && HAL_GetTick() - position_tick[idx] <= CHASSIS_FEEDBACK_FRESH_MS;
    int64_t current = motors[idx].position_feedback;
    if (slot->mode > 2U) return 0U;
    if (slot->mode == 2U) {
        *target = current + delta;
    } else {
        if (!fresh) return 0U;
        if (slot->mode == 0U && !position_target_seen[idx]) return 0U;
        *target = slot->mode == 1U ? delta :
                  last_position_target[idx] + delta;
        delta = *target - current;
    }
    *rpm = delta > 0 ? (float)slot->speed_rpm :
           delta < 0 ? -(float)slot->speed_rpm : 0.0f;
    return 1U;
}

static void remember_request(const float rpm[MOTOR_COUNT])
{
    memcpy(requested_rpm, rpm, sizeof(requested_rpm));
    request_active = 0U;
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        if (rpm[i] != 0.0f) request_active = 1U;
    }
    request_tick = HAL_GetTick();
}

/* idx=0xFF applies a broadcast as ONE candidate body request. Cache writes
 * retain existing wire/ACK semantics, but cannot change the running request.
 * Only a complete, fresh four-wheel group is checked at SYNC_START. */
static HAL_StatusTypeDef submit_motion(uint8_t idx, int32_t value, uint8_t accel,
                                       uint8_t mode, uint8_t sync, uint8_t command,
                                       uint32_t received_tick)
{
    if (idx >= MOTOR_COUNT && idx != 0xFFU) return HAL_ERROR;
    if (command == MOTOR_CMD_SET_POSITION && mode > 2U) return HAL_ERROR;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float candidate[MOTOR_COUNT];
    int64_t targets[MOTOR_COUNT] = {0};
    motion_slot_t slot = {.value=value, .tick=received_tick, .command=command, .mode=mode};
    HAL_StatusTypeDef status = HAL_OK;
    memcpy(candidate, requested_rpm, sizeof(candidate));
    if (!sync) {
        for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
            if (idx != 0xFFU && idx != i) continue;
            slot.speed_rpm = motor_speed_profile[i];
            if (!slot_to_speed(i, &slot, &candidate[i], &targets[i])) { status = HAL_ERROR; break; }
        }
        if (status != HAL_OK || collision_blocks(candidate)) {
            (void)collision_stop();
            status = HAL_ERROR;
            goto done;
        }
        /* Do not leave a stale partial sync group available for a later start. */
        if (staged_mask) { zdt_can_driver_cancel(); staged_mask = 0U; }
    }
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        if (idx != 0xFFU && idx != i) continue;
        status = command == MOTOR_CMD_SET_VELOCITY ?
            zdt_motor_set_speed(motor_slave_addr[i], value, accel, sync) :
            zdt_motor_set_position(motor_slave_addr[i], value, motor_speed_profile[i], accel, mode, sync);
        if (status != HAL_OK) { (void)collision_stop(); break; }
        motor_accel_profile[i] = accel;
        /* Keep the existing feedback meaning: target mirrors the accepted
         * command, including a driver-cache write. The safety request is
         * tracked independently and only becomes active at sync start. */
        if (command == MOTOR_CMD_SET_VELOCITY) {
            int32_t clamped = value > 3000 ? 3000 : value < -3000 ? -3000 : value;
            motors[i].target_velocity = clamped * 10;
        } else motors[i].target_position = value;
        if (!sync) active_wheel_tick[i] = received_tick;
        if (sync) {
            slot.speed_rpm = motor_speed_profile[i];
            staged[i] = slot;
            staged_mask |= (uint8_t)(1U << i);
        }
        else if (command == MOTOR_CMD_SET_POSITION) {
            motors[i].target_position = value;
            last_position_target[i] = targets[i];
            position_target_seen[i] = mode != 2U ||
                (position_seen[i] && HAL_GetTick() - position_tick[i] <= CHASSIS_FEEDBACK_FRESH_MS);
        } else position_target_seen[i] = 0U;
    }
    if (status == HAL_OK && !sync) remember_request(candidate);
done:
    if (primask == 0U) __enable_irq();
    return status;
}

void motor_control_init(void)
{
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
    staged_mask = request_active = physical_estop = local_stop_pending = 0U;
    memset(requested_rpm, 0, sizeof(requested_rpm));
    memset(velocity_seen, 0, sizeof(velocity_seen));
    memset(position_seen, 0, sizeof(position_seen));
    memset(position_target_seen, 0, sizeof(position_target_seen));
    status_poll_index = status_poll_type = speed_poll_index = 0U;
    poll_speed_next = 1U;
    chassis_collision_init();
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

uint8_t motor_process_next_command(Motor_Command_t *cmd, HAL_StatusTypeDef *result)
{
    uint8_t processed;
    uint32_t primask;
    if ((cmd == NULL) || (result == NULL)) { return 0U; }
    primask = __get_PRIMASK();
    __disable_irq();
    /* A stop task cannot run between dequeue and application. */
    processed = motor_fetch_command(cmd, 0U);
    if ((processed == 0U) && (local_sync_start_pending != 0U)) {
        *cmd = local_sync_start;
        local_sync_start_pending = 0U;
        processed = 1U;
    }
    if (processed != 0U) {
        if ((cmd->cmd == MOTOR_CMD_SET_VELOCITY || cmd->cmd == MOTOR_CMD_SET_POSITION ||
             cmd->cmd == MOTOR_CMD_SYNC_START) &&
            HAL_GetTick() - cmd->received_tick > CHASSIS_COMMAND_TIMEOUT_MS) {
            (void)motor_stop_all();
            *result = HAL_TIMEOUT;
        } else *result = motor_apply_command(cmd);
    }
    if (primask == 0U) { __enable_irq(); }
    return processed;
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
    return submit_motion(idx, vel, accel_level, 0U, sync_flag, MOTOR_CMD_SET_VELOCITY, HAL_GetTick());
}

HAL_StatusTypeDef motor_set_position(uint8_t idx, int32_t pos)
{
    if (idx >= MOTOR_COUNT) { return HAL_ERROR; }
    return motor_set_position_ex(idx, pos, motor_accel_profile[idx], 0U, ZDT_SYNC_IMMEDIATE);
}

HAL_StatusTypeDef motor_set_position_ex(uint8_t idx, int32_t pos, uint8_t accel_level, uint8_t mode, uint8_t sync_flag)
{
    return submit_motion(idx, pos, accel_level, mode, sync_flag, MOTOR_CMD_SET_POSITION, HAL_GetTick());
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
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float rpm[MOTOR_COUNT];
    int64_t targets[MOTOR_COUNT] = {0};
    HAL_StatusTypeDef status = HAL_ERROR;
    if (staged_mask != (1U << MOTOR_COUNT) - 1U) goto reject;
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        if (HAL_GetTick() - staged[i].tick > CHASSIS_COMMAND_TIMEOUT_MS ||
            !slot_to_speed(i, &staged[i], &rpm[i], &targets[i])) goto reject;
    }
    if (collision_blocks(rpm)) goto reject;
    status = zdt_trigger_sync_start();
    if (status != HAL_OK) goto reject;
    remember_request(rpm);
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        active_wheel_tick[i] = staged[i].tick;
        position_target_seen[i] = staged[i].command == MOTOR_CMD_SET_POSITION &&
            (staged[i].mode != 2U || (position_seen[i] &&
             HAL_GetTick() - position_tick[i] <= CHASSIS_FEEDBACK_FRESH_MS));
        if (staged[i].command == MOTOR_CMD_SET_POSITION) {
            last_position_target[i] = targets[i];
            motors[i].target_position = staged[i].value;
        }
    }
    staged_mask = 0U;
    goto done;
reject:
    (void)collision_stop();
done:
    if (primask == 0U) __enable_irq();
    return status;
}

HAL_StatusTypeDef motor_send_multi_cmd(const uint8_t *cmd_stream, uint16_t stream_len)
{
    /* Opaque AA motion cannot supply an atomic, decoded chassis request.
     * Use the existing per-wheel cache + SYNC_START interface instead. */
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

    request_active = staged_mask = 0U;
    memset(requested_rpm, 0, sizeof(requested_rpm));
    memset(position_target_seen, 0, sizeof(position_target_seen));
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
    position_tick[idx] = HAL_GetTick();
    position_seen[idx] = 1U;
}

static void motor_velocity_read_cb(uint8_t slave, int32_t vel)
{
    uint8_t idx;

    idx = motor_find_index_by_slave(slave);
    if (idx >= MOTOR_COUNT) {
        return;
    }

    motors[idx].current_velocity = vel;
    velocity_tick[idx] = HAL_GetTick();
    velocity_seen[idx] = 1U;
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
        /* At one completed query / 20 ms, velocities recur every 160 ms,
         * allowing a fresh four-wheel snapshot within the 200 ms limit.
         * Missing feedback is ignored; driver read retries remain bounded. */
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
            status = submit_motion(cmd->motor_idx, cmd->value, cmd->param0, 0U, cmd->param1, cmd->cmd,
                                   cmd->received_tick);
            break;

        case MOTOR_CMD_ESTOP:
            status = motor_stop_all();
            break;

        case MOTOR_CMD_SET_POSITION:
            status = submit_motion(cmd->motor_idx, cmd->value, cmd->param0,
                                   (uint8_t)(cmd->param1 & 0x03U),
                                   (uint8_t)((cmd->param1 >> 2) & 1U), cmd->cmd, cmd->received_tick);
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
