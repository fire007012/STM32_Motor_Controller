#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_transport.h"
#include "can_protocol.h"
#include "motor_control.h"
#include "zdt_can_driver.h"
#include "zdt_status.h"
#include "chassis_safety_config.h"

static CAN_TypeDef can_registers[2];
CAN_HandleTypeDef hcan1 = {0U, &can_registers[0]}, hcan2 = {1U, &can_registers[1]};
TIM_HandleTypeDef htim6;
uint32_t test_primask;
static uint32_t tick;
static unsigned free_mailboxes[2];
static unsigned fail_next[2];
static unsigned busy_next[2];
static unsigned aborted[2];
static CAN_TxFrame_t sent[2][512];
static unsigned sent_count[2];
static CAN_RxHeaderTypeDef rx_headers[2][32];
static uint8_t rx_data[2][32][8];
static unsigned rx_head[2], rx_tail[2];
static unsigned servo_count;
static Motor_Command_t command_queue[MOTOR_CMD_QUEUE_LENGTH];
static unsigned command_head, command_tail, command_count;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *bus);
uint32_t HAL_GetTick(void) { return tick; }
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *bus)
{ (void)bus; return HAL_CAN_STATE_LISTENING; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *bus)
{ return free_mailboxes[bus->index]; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *bus, uint32_t mailboxes)
{ (void)mailboxes; return (free_mailboxes[bus->index] == 3U) ? 0U : 1U; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                      const uint8_t data[8], uint32_t *mailbox)
{
    unsigned i = bus->index;
    assert(test_primask == 1U);
    if (fail_next[i] != 0U) { fail_next[i]--; return HAL_ERROR; }
    if (busy_next[i] != 0U) { busy_next[i]--; return HAL_BUSY; }
    assert(free_mailboxes[i] != 0U);
    assert(sent_count[i] < 512U);
    free_mailboxes[i]--;
    sent[i][sent_count[i]].header = *header;
    memcpy(sent[i][sent_count[i]++].data, data, 8U);
    *mailbox = 1U << (2U - free_mailboxes[i]);
    bus->Instance->TSR |= CAN_TSR_TXOK0 << (8U * (2U - free_mailboxes[i]));
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *bus, uint32_t mailboxes)
{
    assert(mailboxes != 0U && (mailboxes & ~7U) == 0U);
    aborted[bus->index]++;
    free_mailboxes[bus->index] = 3U;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *bus, uint32_t fifo)
{ (void)fifo; return rx_tail[bus->index] - rx_head[bus->index]; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *bus, uint32_t fifo,
                                      CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    unsigned i = bus->index, head = rx_head[i]++;
    (void)fifo;
    *header = rx_headers[i][head];
    memcpy(data, rx_data[i][head], 8U);
    return HAL_OK;
}
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *bus)
{
    if (bus->Instance->MSR & CAN_MSR_ERRI) {
        bus->Instance->ESR &= ~0x70U;
        bus->Instance->MSR &= ~CAN_MSR_ERRI;
    }
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer) { (void)timer; }
static unsigned arm_servo_count;
uint8_t ArmServo_HandleCanCommand(const uint8_t data[8]) { (void)data; arm_servo_count++; return 0U; }
void Servo_AcquireRemoteControl(void) {}
void Servo_ReleaseLocalControl(void) {}
uint8_t Servo_HandlePositionCanCommand(const uint8_t data[8]) { (void)data; servo_count++; return 0U; }
void Servo_HandleCanCommand(const uint8_t data[8]) { (void)data; servo_count++; }
void Error_Handler(void) { abort(); }

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const void *attr)
{
    (void)attr;
    assert(count == MOTOR_CMD_QUEUE_LENGTH && size == sizeof(Motor_Command_t));
    return command_queue;
}
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message, uint8_t priority, uint32_t timeout)
{
    (void)priority;
    assert(queue == command_queue && timeout == 0U);
    if (command_count == MOTOR_CMD_QUEUE_LENGTH) { return osErrorResource; }
    command_queue[command_tail] = *(const Motor_Command_t *)message;
    command_tail = (command_tail + 1U) % MOTOR_CMD_QUEUE_LENGTH;
    command_count++;
    return osOK;
}
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message, uint8_t *priority, uint32_t timeout)
{
    (void)priority;
    assert(queue == command_queue && timeout == 0U);
    if (command_count == 0U) { return osErrorResource; }
    *(Motor_Command_t *)message = command_queue[command_head];
    command_head = (command_head + 1U) % MOTOR_CMD_QUEUE_LENGTH;
    command_count--;
    return osOK;
}
osStatus_t osMessageQueueReset(osMessageQueueId_t queue)
{
    assert(queue == command_queue);
    command_head = command_tail = command_count = 0U;
    return osOK;
}

static void reset(void)
{
    const uint8_t addresses[4] = {1U, 2U, 3U, 4U};
    test_primask = tick = 0U;
    memset(sent_count, 0, sizeof(sent_count));
    memset(fail_next, 0, sizeof(fail_next));
    memset(busy_next, 0, sizeof(busy_next));
    memset(aborted, 0, sizeof(aborted));
    memset(can_registers, 0, sizeof(can_registers));
    memset(rx_head, 0, sizeof(rx_head));
    memset(rx_tail, 0, sizeof(rx_tail));
    memset(motors, 0, sizeof(motors));
    free_mailboxes[0] = free_mailboxes[1] = 0U;
    command_head = command_tail = command_count = servo_count = 0U;
    can_transport_init(&hcan1, &hcan2);
    zdt_can_driver_init(&hcan2);
    motor_control_init();
    motor_set_ros_alive();
    assert(motor_configure_addresses(addresses, MOTOR_COUNT) == HAL_OK);
}

static void drain(void)
{
    unsigned i;
    for (i = 0U; i < 256U; i++) {
        free_mailboxes[0] = free_mailboxes[1] = 3U;
        can_transport_service();
        assert(test_primask == 0U);
    }
}

static void expect_frame(unsigned bus, unsigned index, uint32_t id, const uint8_t *data, unsigned length)
{
    assert(index < sent_count[bus]);
    assert(sent[bus][index].header.IDE == CAN_ID_EXT);
    assert(sent[bus][index].header.ExtId == id);
    assert(sent[bus][index].header.DLC == length);
    assert(memcmp(sent[bus][index].data, data, length) == 0);
}

static void reply(uint8_t address, const uint8_t *data, uint8_t length)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t packet[8] = {0};
    header.IDE = CAN_ID_EXT;
    header.ExtId = (uint32_t)address << 8;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    memcpy(packet, data, length);
    zdt_can_driver_process_response(header, packet);
}

/* Legacy single-command checks now wait for the terminal result, rather than
 * confusing consumption of the queue with completion of native execution. */
static uint8_t pump_one_command(Motor_Command_t *cmd, HAL_StatusTypeDef *result)
{
    if (!motor_process_next_command(cmd, result)) return 0U;
    unsigned seen = sent_count[1];
    for (unsigned step=0U; step<32U; ++step) {
        drain();
        while (seen<sent_count[1]) {
            CAN_TxFrame_t *f = &sent[1][seen++];
            uint8_t func = f->data[0];
            if (f->header.ExtId && (func==0xF6U || func==0xFBU || func==0xFEU))
                reply((uint8_t)(f->header.ExtId>>8),(uint8_t[]){func,2,0x6B},3);
            else if (func==0xFFU)
                reply(motor_get_address(0),(uint8_t[]){func,2,0x6B},3);
        }
        zdt_can_driver_timeout_poll();
        if (motor_take_command_result(cmd,result)) return 1U;
        tick++;
        /* Do not eat another host command while completing this fixture. */
        if (step==CHASSIS_CONTROL_PERIOD_MS-1U) {
            (void)motor_process_next_command(cmd,result);
        }
    }
    assert(!"command did not reach a terminal result");
    return 0U;
}

static void test_startup_and_lengths(void)
{
    unsigned address;
    const uint8_t enable[] = {0xF3, 0xAB, 1, 0, 0x6B};
    const uint8_t ack[] = {0xF3, 2, 0x6B};
    reset();
    assert(motor_enable_all() == HAL_OK);
    for (address = 1U; address <= MOTOR_COUNT; address++) {
        drain();
        expect_frame(1, address - 1U, address << 8, enable, sizeof(enable));
        reply((uint8_t)address, ack, sizeof(ack));
    }
    assert(sent_count[1] == MOTOR_COUNT);
    assert(zdt_can_driver_is_busy() == 0U);

    reset();
    assert(zdt_motor_stop(0, 0) == HAL_OK);
    assert(zdt_trigger_sync_start() == HAL_OK);
    assert(zdt_read_motor_status(1, NULL) == HAL_OK);
    drain();
    expect_frame(1, 0, 0, (uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
    expect_frame(1, 1, 0, (uint8_t[]){0xFF, 0x66, 0x6B}, 3);
    expect_frame(1, 2, 0x100, (uint8_t[]){0x3A, 0x6B}, 2);
    puts("PASS startup enables only four chassis motors; exact short-command DLC");
}

static void test_ros_bus_fault_backoff(void)
{
    CAN_TxHeaderTypeDef header = {.StdId = 0x101U, .DLC = 8U};
    const uint8_t data[8] = {1U};
    unsigned rejected = 0U;
    reset();
    /* Reproduce continuous telemetry with no ACK: every requested TX leaves
     * its mailbox without TXOK, while producers still run every 20 ms. */
    for (tick = 0U; tick < 1000U; tick++) {
        free_mailboxes[0] = free_mailboxes[1] = 3U;
        can_registers[0].TSR = 0U;
        if ((tick % 20U) == 0U) {
            if (can_transport_send(&hcan1, &header, data) == HAL_BUSY) rejected++;
            assert(can_transport_send(&hcan2, &header, data) == HAL_OK);
        }
        can_transport_service();
    }
    unsigned failed_attempts = sent_count[0];
    assert(failed_attempts > 0U && failed_attempts <= 10U && rejected > 0U);
    assert(can_transport_get_failure_count(&hcan1) == failed_attempts);
    assert(sent_count[1] == 50U); /* A broken ROS link cannot throttle CAN2. */
    tick += CAN_ROS_TX_BACKOFF_MS;
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    drain(); /* Now TXOK is set by HAL; normal traffic resumes. */
    assert(sent_count[0] > failed_attempts);
    unsigned recovered = sent_count[0];
    for (unsigned i = 0U; i < 50U; i++) {
        assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
        drain();
    }
    assert(sent_count[0] == recovered + 50U);
    assert(can_transport_get_failure_count(&hcan1) == failed_attempts);

    /* Even single-shot TX can get stuck before it reaches the wire. Bound
     * that wait, cancel stale queued data and preserve the caller's IRQ mask. */
    reset();
    tick = UINT32_MAX - 10U;
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(sent_count[0] == 1U);
    tick += CAN_ROS_TX_TIMEOUT_MS - 1U;
    can_transport_service();
    assert(aborted[0] == 0U && sent_count[0] == 1U);
    tick++;
    test_primask = 1U;
    can_transport_service();
    assert(test_primask == 1U && aborted[0] == 1U);
    test_primask = 0U;
    assert(can_transport_get_failure_count(&hcan1) == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS - 1U;
    assert(can_transport_send(&hcan1, &header, data) == HAL_BUSY);
    tick++;
    drain();
    assert(sent_count[0] == 1U); /* Canceled stale traffic is never replayed. */
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    drain();
    assert(sent_count[0] == 2U);

    reset();
    fail_next[0] = 1U;
    free_mailboxes[0] = 3U;
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    for (unsigned i = 0U; i < 100U; i++) can_transport_service();
    assert(sent_count[0] == 0U && can_transport_get_failure_count(&hcan1) == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 1U); /* HAL submission failure is retried after backoff. */

    reset();
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    header.StdId = 0x102U;
    assert(can_transport_send(&hcan1, &header, data) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    can_registers[0].TSR = CAN_TSR_ALST0;
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(sent_count[0] == 2U && sent[0][1].header.StdId == 0x101U);
    assert(can_transport_get_failure_count(&hcan1) == 0U);
    drain();
    assert(sent_count[0] == 3U && sent[0][2].header.StdId == 0x102U);
    puts("PASS ROS no-ACK attempt bound, CAN2 independence, recovery, stuck-mailbox abort, tick wrap and HAL backoff");
}

static void test_x_motion_encoding(void)
{
    reset();
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    assert(zdt_motor_set_speed(1, 100, 50, 0) == HAL_OK);
    assert(zdt_motor_set_speed(2, -120, 50, 0) == HAL_OK);
    assert(zdt_motor_set_speed(3, INT32_MIN, 0, 1) == HAL_OK);
    assert(zdt_motor_set_speed(4, 65536, 0, 0) == HAL_OK);
    drain();
    expect_frame(1, 0, 0x100, (uint8_t[]){0xF6, 0, 0, 50, 3, 0xE8, 0, 0x6B}, 8);
    expect_frame(1, 1, 0x200, (uint8_t[]){0xF6, 1, 0, 50, 4, 0xB0, 0, 0x6B}, 8);
    expect_frame(1, 2, 0x300, (uint8_t[]){0xF6, 1, 0, 0, 0x75, 0x30, 1, 0x6B}, 8);
    expect_frame(1, 3, 0x400, (uint8_t[]){0xF6, 0, 0, 0, 0x75, 0x30, 0, 0x6B}, 8);

    reset();
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    assert(zdt_motor_set_position(1, 3200, 100, 50, 0, 0) == HAL_OK);
    assert(zdt_motor_set_position(2, INT32_MIN, 100, 50, 2, 1) == HAL_OK);
    assert(zdt_motor_set_position(1, 1, 100, 50, 3, 0) == HAL_ERROR);
    drain();
    expect_frame(1, 0, 0x100, (uint8_t[]){0xFB, 0, 3, 0xE8, 0, 0, 0x0E, 0x10}, 8);
    expect_frame(1, 1, 0x101, (uint8_t[]){0xFB, 0, 0, 0x6B}, 4);
    expect_frame(1, 2, 0x200, (uint8_t[]){0xFB, 1, 3, 0xE8, 0x90, 0, 0, 0}, 8);
    expect_frame(1, 3, 0x201, (uint8_t[]){0xFB, 2, 1, 0x6B}, 4);
    puts("PASS X F6/FB wire bytes, tail DLC, signed extremes and speed clamping");
}

static void test_mailbox_backpressure(void)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    unsigned i;
    reset();
    header.IDE = CAN_ID_EXT;
    header.DLC = 2U;
    for (i = 0; i < 12; i++) {
        header.ExtId = 0x100U + i;
        data[0] = (uint8_t)i;
        assert(can_transport_send(&hcan2, &header, data) == HAL_OK);
    }
    can_transport_service();
    assert(sent_count[1] == 0U);
    free_mailboxes[1] = 3U;
    can_transport_service();
    assert(sent_count[1] == 3U);
    fail_next[1] = 1U;
    free_mailboxes[1] = 3U;
    can_transport_service();
    assert(sent_count[1] == 3U);
    drain();
    assert(sent_count[1] == 12U);
    for (i = 0; i < 12; i++) {
        assert(sent[1][i].header.ExtId == 0x100U + i);
        assert(sent[1][i].data[0] == i);
    }
    assert(can_transport_get_failure_count(&hcan2) == 1U);

    reset();
    for (i = 0; i < CAN_TX_QUEUE_LENGTH - 1U; i++) {
        assert(can_transport_send(&hcan2, &header, data) == HAL_OK);
    }
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    Motor_Command_t cmd = {.cmd = MOTOR_CMD_SET_POSITION, .motor_idx = 0, .value = 3200, .param1 = 2};
    assert(motor_apply_command(&cmd) == HAL_BUSY);
    assert(motors[0].target_position == 0);
    drain();
    assert(sent_count[1] == CAN_TX_QUEUE_LENGTH - 1U); /* atomic rejection preserves backlog */
    assert(zdt_can_driver_get_tx_fail_count() == 1U);
    puts("PASS mailbox exhaustion, HAL failure retry, atomic batches and failure propagation");
}

static void test_routing_and_mapping(void)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {1, 3, 100, 0, 0, 0, 0, 0};
    Motor_Command_t cmd;
    unsigned ide;
    reset();
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    for (ide = 0; ide < 2; ide++) {
        header.IDE = (ide == 0U) ? CAN_ID_STD : CAN_ID_EXT;
        header.StdId = header.ExtId = 0x201;
        header.DLC = 8;
        CAN1_RxCallback(header, data);
        assert(motor_fetch_command(&cmd, 0) == 1U);
        assert(cmd.motor_idx == 3U && cmd.param0 == 50U);
        assert(motor_apply_command(&cmd) == HAL_OK);
    }
    drain();
    expect_frame(1, 0, 0x400, (uint8_t[]){0xF6, 0, 0, 50, 3, 0xE8, 0, 0x6B}, 8);
    expect_frame(1, 1, 0x400, (uint8_t[]){0xF6, 0, 0, 50, 3, 0xE8, 0, 0x6B}, 8);
    tick = 501;
    header.DLC = 3;
    CAN1_RxCallback(header, data);
    assert(command_count == 0U && motor_is_ros_timeout(500) == 1U);
    header.DLC = 8;
    header.RTR = CAN_RTR_REMOTE;
    CAN1_RxCallback(header, data);
    assert(command_count == 0U && motor_is_ros_timeout(500) == 1U);
    header.RTR = CAN_RTR_DATA;
    data[1] = 0xFE;
    CAN1_RxCallback(header, data);
    assert(command_count == 0U);

    cmd = (Motor_Command_t){.cmd = MOTOR_CMD_SET_ADDRESS_MAP, .value = 0x04030201};
    assert(motor_apply_command(&cmd) == HAL_OK && motor_get_address(3) == 4);
    assert(motor_get_address(4) == 0);
    cmd.value = 0x04030301;
    assert(motor_apply_command(&cmd) == HAL_ERROR && motor_get_address(3) == 4);
    cmd.value = 0x04030001;
    assert(motor_apply_command(&cmd) == HAL_ERROR && motor_get_address(1) == 2);
    puts("PASS standard/extended 0x201 routing, fourth chassis motor, malformed frames and address-map validation");
}

static void test_sync_timeout_and_stop(void)
{
    reset();
    assert(zdt_motor_set_position(1, 3200, 100, 50, 0, 1) == HAL_OK);
    assert(zdt_motor_set_position(2, 3200, 100, 50, 0, 1) == HAL_OK);
    assert(zdt_trigger_sync_start() == HAL_OK);
    drain();
    assert(sent_count[1] == 2U);
    tick = 50;
    zdt_can_driver_timeout_poll(); /* lost ACK never replays relative motion */
    drain();
    assert(sent_count[1] == 4U);
    assert(sent[1][2].header.ExtId == 0x200);
    reply(2, (uint8_t[]){0xFB, 2, 0x6B}, 3);
    drain();
    expect_frame(1, 4, 0, (uint8_t[]){0xFF, 0x66, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 0U);
    assert(zdt_can_driver_get_timeout_drop_count() == 1U);

    reset();
    assert(zdt_read_motor_status(1, NULL) == HAL_OK);
    assert(zdt_read_realtime_position(1, NULL) == HAL_BUSY);
    for (unsigned i = 0; i < 4; i++) {
        drain();
        tick += 50;
        zdt_can_driver_timeout_poll();
    }
    drain();
    assert(sent_count[1] == 4 && zdt_can_driver_get_timeout_drop_count() == 1);

    reset();
    assert(zdt_read_motor_status(2, NULL) == HAL_OK);
    assert(zdt_motor_set_speed(1, 100, 50, 0) == HAL_OK);
    drain();
    assert(sent_count[1] == 2U); /* motion preempts the missing motor's read */
    reply(2, (uint8_t[]){0x3A, 0x83, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 1U);
    reply(1, (uint8_t[]){0xF6, 2, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 0U);

    reset();
    Motor_Command_t command = {.cmd = MOTOR_CMD_SET_VELOCITY, .motor_idx = 0, .value = 100};
    assert(motor_enqueue_command_from_isr(&command) == 1U);
    assert(motor_set_velocity(0, 100) == HAL_OK);
    assert(motor_set_velocity(1, 100) == HAL_OK);
    assert(motor_stop_all() == HAL_OK);
    assert(command_count == 0U && aborted[1] == 1U);
    tick = 1000;
    zdt_can_driver_timeout_poll();
    drain();
    assert(sent_count[1] == 1U);
    expect_frame(1, 0, 0, (uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
    assert(motors[0].target_velocity == 0);
    puts("PASS sync ordering, no motion replay, bounded read retry and stop cancels stale work");
}

static void test_local_sync_ordering_and_ros_gate(void)
{
    Motor_Command_t queued = {
        .cmd = MOTOR_CMD_SET_POSITION,
        .motor_idx = 0U,
        .value = 3200,
        .param0 = 50U,
        .param1 = 4U,
        .source = MOTOR_CMD_SOURCE_REMOTE
    };
    Motor_Command_t executed = {0};
    HAL_StatusTypeDef result;

    reset();
    motor_set_ros_startup_grace();
    tick = 501U;
    assert(motor_is_ros_ready(500U) == 0U);
    assert(motor_request_local_sync_start(0x1234U) == 0U);

    motor_set_ros_alive();
    assert(motor_is_ros_ready(500U) == 1U);
    queued.param1 = 6U; /* relative-current, cached */
    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        queued.motor_idx = i;
        assert(motor_enqueue_command_from_isr(&queued) == 1U);
    }
    assert(motor_request_local_sync_start(0x1234U) == 1U);

    for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
        assert(pump_one_command(&executed, &result) == 1U && result == HAL_OK);
        assert(executed.cmd == MOTOR_CMD_SET_POSITION);
        assert(executed.source == MOTOR_CMD_SOURCE_REMOTE);
    }
    assert(pump_one_command(&executed, &result) == 1U);
    assert(executed.cmd == MOTOR_CMD_SYNC_START && result == HAL_OK);
    assert(executed.source == MOTOR_CMD_SOURCE_LOCAL_BUTTON);
    assert(executed.seq == 0x1234U);

    assert(motor_request_local_sync_start(0x1235U) == 1U);
    assert(motor_stop_all() == HAL_OK);
    assert(pump_one_command(&executed, &result) == 0U);
    puts("PASS local sync waits behind ROS commands, requires ROS activity and is canceled by stop");
}

static void test_polling_and_faults(void)
{
    const uint8_t functions[6] = {0x35,0x3A,0x35,0x36,0x35,0x3A};
    const uint8_t addresses[6] = {1,1,2,1,3,2};
    zdt_estop_event_t event;
    reset();
    for (unsigned i = 0; i < 6; i++) {
        motor_update_status(1);
        motor_update_status(1); /* cannot consume a second slot while busy */
        drain();
        assert(sent_count[1] == i+1U);
        assert(sent[1][i].data[0] == functions[i]);
        assert(sent[1][i].header.ExtId == (uint32_t)addresses[i]<<8);
        if (functions[i]==0x35U) {
            reply(addresses[i],(uint8_t[]){0x35,1,3,0xE8,0x6B},5);
            assert(motors[addresses[i]-1U].current_velocity == -1000);
            assert(zdt_feedback_diagnostics[addresses[i]-1U].speed_updates == 1U);
        } else if (functions[i]==0x36U) {
            reply(addresses[i],(uint8_t[]){0x36,0,0,0,0x0E,0x10,0x6B},7);
            assert(motors[0].position_feedback == 3600);
        } else if(i==1U) {
            reply(2,(uint8_t[]){0x3A,0x83,0x6B},3);
            reply(1,(uint8_t[]){0x3A,0x83,0},3);
            assert(zdt_can_driver_is_busy());
            assert(zdt_feedback_diagnostics[0].status_updates == 0U);
            reply(1,(uint8_t[]){0x3A,0x83,0x6B},3);
            assert(motors[0].status_word==0x83 && !motors[0].fault_flag);
            assert(zdt_feedback_diagnostics[0].status_updates == 1U);
            assert(!zdt_status_take_estop_event(&event));
        } else {
            reply(2,(uint8_t[]){0x3A,0x89,0x6B},3);
            assert(zdt_status_take_estop_event(&event) && event.motor_idx==1U);
        }
    }
    puts("PASS bounded speed-priority polling, response validation, default 0x83 healthy and stall protection");
}

static void inject(CAN_HandleTypeDef *bus, uint32_t id, const uint8_t *data, uint8_t length)
{
    unsigned i = bus->index, tail = rx_tail[i]++;
    CAN_RxHeaderTypeDef *header = &rx_headers[i][tail];
    memset(header, 0, sizeof(*header));
    header->IDE = CAN_ID_EXT;
    header->ExtId = id;
    header->DLC = length;
    memset(rx_data[i][tail], 0, 8U);
    memcpy(rx_data[i][tail], data, length);
}

static void test_bidirectional_bridge_and_rejection(void)
{
    reset();
    for (unsigned i = 0; i < 6; i++) {
        inject(&hcan1, 0x100, (uint8_t[]){0xF3, 0xAB, 1, 0, 0x6B}, 5);
        inject(&hcan2, 0x100, (uint8_t[]){0xF3, 2, 0x6B}, 3);
    }
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(rx_head[0] == 6 && rx_head[1] == 6);
    assert(sent_count[0] == 0 && sent_count[1] == 0); /* no HAL TX from RX callback */
    drain();
    assert(sent_count[0] == 6 && sent_count[1] == 6);
    expect_frame(0, 0, 0x100, (uint8_t[]){0xF3, 2, 0x6B}, 3);
    expect_frame(1, 0, 0x100, (uint8_t[]){0xF3, 0xAB, 1, 0, 0x6B}, 5);

    reset();
    CAN_RxHeaderTypeDef header = {.StdId = 0x100, .IDE = CAN_ID_STD, .DLC = 8};
    uint8_t data[8] = {1, 0, 100, 0, 0, 0, 50, 0};
    for (unsigned i = 0; i <= MOTOR_CMD_QUEUE_LENGTH; i++) {
        CAN1_RxCallback(header, data);
    }
    drain();
    assert(command_count == MOTOR_CMD_QUEUE_LENGTH && sent_count[0] == 1);
    assert(sent[0][0].header.StdId == 0x102 && sent[0][0].data[3] == 1);
    puts("PASS bidirectional queued bridge, FIFO draining and queue-full negative ACK");
}

static void test_aa_bounds_and_length(void)
{
    reset();
    assert(zdt_send_multi_motor_command((uint8_t[]){1, 0x36, 0x6B}, 3) == HAL_OK);
    assert(zdt_send_multi_motor_command((uint8_t[]){1}, UINT16_MAX) == HAL_ERROR);
    drain();
    expect_frame(1, 0, 0, (uint8_t[]){0xAA, 0, 8, 1, 0x36, 0x6B, 0x6B}, 7);
    puts("PASS AA total length and oversized-stream rejection before copying");
}

static void test_estop_when_command_queue_full(void)
{
    Motor_Command_t command = {.cmd = MOTOR_CMD_SET_VELOCITY, .motor_idx = 0, .value = 100};
    Motor_Command_t stop = {.cmd = MOTOR_CMD_ESTOP, .motor_idx = 0xFF};
    Motor_Command_t next;
    reset();
    for (unsigned i = 0; i < MOTOR_CMD_QUEUE_LENGTH; i++) {
        assert(motor_enqueue_command_from_isr(&command) == 1U);
    }
    assert(motor_enqueue_command_from_isr(&stop) == 1U);
    assert(motor_fetch_command(&next, 0) == 1U && next.cmd == MOTOR_CMD_ESTOP);
    assert(motor_apply_command(&next) == HAL_OK);
    assert(motor_fetch_command(&next, 0) == 0U);
    drain();
    expect_frame(1, 0, 0, (uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
    puts("PASS emergency stop bypasses a full command queue and clears older moves");
}

static void test_pending_survives_transport_full(void)
{
    CAN_TxHeaderTypeDef header = {.ExtId = 0x900, .IDE = CAN_ID_EXT, .DLC = 2};
    uint8_t filler[8] = {0x36, 0x6B};
    reset();
    assert(zdt_motor_set_speed(1, 100, 50, 0) == HAL_OK);
    drain();
    for (unsigned i = 0; i < CAN_TX_QUEUE_LENGTH; i++) {
        assert(can_transport_send(&hcan2, &header, filler) == HAL_OK);
    }
    assert(zdt_motor_set_speed(2, 200, 50, 0) == HAL_OK);
    reply(1, (uint8_t[]){0xF6, 2, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 1U);
    drain();
    zdt_can_driver_timeout_poll();
    drain();
    expect_frame(1, CAN_TX_QUEUE_LENGTH + 1U, 0x200,
                 (uint8_t[]){0xF6, 0, 0, 50, 7, 0xD0, 0, 0x6B}, 8);
    assert(zdt_can_driver_get_timeout_drop_count() == 0U);
    puts("PASS accepted driver request survives temporary transport queue exhaustion");
}

static void test_response_timeout_waits_for_transmission(void)
{
    reset();
    assert(zdt_read_motor_status(1, NULL) == HAL_OK);
    for (unsigned i = 0; i < 10; i++) {
        tick += 50;
        zdt_can_driver_timeout_poll();
        can_transport_service(); /* all hardware mailboxes unavailable */
    }
    assert(zdt_can_driver_is_busy() == 1U);
    assert(zdt_can_driver_get_timeout_drop_count() == 0U);
    drain();
    assert(sent_count[1] == 1U); /* no duplicate reads before the first send */
    reply(1, (uint8_t[]){0x3A, 0x83, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 0U);

    reset();
    assert(zdt_read_motor_status(1, NULL) == HAL_OK);
    free_mailboxes[1] = 3U;
    can_transport_service(); /* Accepted by HAL but still pending on the wire. */
    tick = 500U;
    zdt_can_driver_timeout_poll();
    assert(zdt_can_driver_is_busy() == 1U && sent_count[1] == 1U);
    assert(zdt_can_driver_get_timeout_drop_count() == 0U);
    free_mailboxes[1] = 3U;
    can_transport_poll_completions(&hcan2);
    tick = 549U;
    zdt_can_driver_timeout_poll();
    drain();
    assert(sent_count[1] == 1U);
    tick = 550U;
    zdt_can_driver_timeout_poll();
    drain();
    assert(sent_count[1] == 2U);

    reset();
    assert(zdt_read_motor_status(1, NULL) == HAL_OK);
    free_mailboxes[1] = 3U;
    can_transport_service();
    free_mailboxes[1] = 3U; /* TX finishes, but the task has not polled it yet. */
    reply(1, (uint8_t[]){0x3A, 0x83, 0x6B}, 3);
    assert(zdt_can_driver_is_busy() == 0U); /* RX IRQ detects completion itself. */
    puts("PASS response timeout starts after actual transmission, not software enqueue");
}

static void test_continuous_multi_motor_bursts(void)
{
    Motor_Command_t command = {.cmd = MOTOR_CMD_SET_VELOCITY, .value = 100, .param0 = 50};
    Motor_Command_t executed;
    HAL_StatusTypeDef result;
    Motor_CommStats_t before, after;
    reset();
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    motor_get_comm_stats(&before);
    /* Four motors at 50 Hz: 200 commands/s, beyond the previous 100/s drain. */
    for (unsigned cycle = 0; cycle < 50; cycle++) {
        tick = cycle * 20U;
        motor_set_ros_alive();
        for (unsigned motor = 0; motor < MOTOR_COUNT; motor++) {
            command.motor_idx = (uint8_t)motor;
            assert(motor_enqueue_command_from_isr(&command) == 1U);
        }
        for (unsigned i=0; i<MOTOR_COUNT; ++i)
            assert(pump_one_command(&executed,&result) && result==HAL_OK);
        assert(command_count==0U);
    }
    motor_get_comm_stats(&after);
    assert(after.exec_ok_count - before.exec_ok_count == 50U * MOTOR_COUNT);
    assert(after.exec_fail_count == before.exec_fail_count && sent_count[1] == 50U * MOTOR_COUNT);
    puts("PASS four-motor 50 Hz command bursts without queue accumulation");
}

static void test_fault_event_handoff_and_irq_mask(void)
{
    zdt_estop_event_t event;
    reset();
    zdt_status_update_speed(0, -1000);
    zdt_status_update_status(0, 0x89);
    test_primask = 1U;
    assert(zdt_status_take_estop_event(&event) == 1U);
    assert(test_primask == 1U && event.motor_idx == 0U && event.speed_rpm == -1000);
    test_primask = 0U;
    zdt_status_update_speed(1, 2000);
    zdt_status_update_status(1, 0x89);
    assert(zdt_status_take_estop_event(&event) == 1U);
    assert(test_primask == 0U && event.motor_idx == 1U && event.speed_rpm == 2000);
    assert(zdt_status_take_estop_event(&event) == 0U && test_primask == 0U);
    puts("PASS atomic fault-event handoff and preserved caller interrupt mask");
}

static void test_local_pwm_ack_routing(void)
{
    reset();
    arm_servo_count = 0U;
    CAN_RxHeaderTypeDef header = {.ExtId = 0x701, .IDE = CAN_ID_EXT, .DLC = 8};
    uint8_t data[8] = {0x21,3,150,0,0,0,42,0};
    for (unsigned i = 0; i < 7; ++i) data[7] += data[i];
    CAN1_RxCallback(header, data);drain();
    assert(arm_servo_count == 1 && sent_count[1] == 0);
    expect_frame(0,0,0x702,(uint8_t[]){0xA1,42,0,0x21,3,0,0,0xEF},8);
    header.DLC=7;CAN1_RxCallback(header,data);drain();
    assert(arm_servo_count==1 && sent_count[0]==1 && sent_count[1]==0);
    header.DLC=8;header.ExtId=0x700;data[0]=0x22;
    CAN1_RxCallback(header,data);drain();
    assert(servo_count==1 && sent_count[1]==0 && sent[0][1].header.ExtId==0x703);
    header.ExtId=0x500;CAN1_RxCallback(header,data);header.ExtId=0x600;CAN1_RxCallback(header,data);drain();
    assert(sent_count[1]==0);
    header.IDE=CAN_ID_STD;header.StdId=0x201;data[0]=1;data[1]=4;
    CAN1_RxCallback(header,data);assert(command_count==0);
    puts("PASS local paired PWM ACK routing, reserved channel rejection and removed Y42 IDs never forwarded");
}


static void expect_actual_stop(void)
{
    drain();
    assert(sent_count[1] > 0U);
    expect_frame(1, sent_count[1]-1U, 0U, (uint8_t[]){0xFE,0x98,0,0x6B},4);
    for (uint8_t i=0U; i<4U; ++i) assert(motors[i].target_velocity == 0);
    for (unsigned i=0U; i<sent_count[0]; ++i)
        assert(!(sent[0][i].header.StdId == ROS_CAN_STATUS_ID && sent[0][i].data[1] == 6U));
}

static void test_ros_owned_parking(void)
{
    Motor_Command_t in={.cmd=MOTOR_CMD_SET_VELOCITY,.motor_idx=2,.value=-5,.param0=50,.seq=400}, out;
    HAL_StatusTypeDef status;
    reset();
    assert(motor_enqueue_command_from_isr(&in));
    assert(pump_one_command(&out,&status) && status==HAL_OK);
    assert(sent_count[1]==1 && motors[2].target_velocity==-50);
    /* Heartbeats do not impose an independent running-command lease. */
    for(unsigned n=0;n<10;++n) { tick+=100; motor_set_ros_alive(); motor_safety_poll(); }
    drain(); assert(sent_count[1]==1 && motors[2].target_velocity==-50);
    /* Parameter/cache/stale-queue rejections report failure without parking. */
    in.cmd=MOTOR_CMD_SYNC_START;
    assert(motor_enqueue_command_from_isr(&in));
    assert(pump_one_command(&out,&status) && status==HAL_ERROR);
    assert(motor_last_failure.reason==MOTOR_REASON_CACHE_INCOMPLETE);
    in.cmd=MOTOR_CMD_SET_VELOCITY; in.motor_idx=9;
    assert(motor_enqueue_command_from_isr(&in));
    assert(pump_one_command(&out,&status) && status==HAL_ERROR);
    in.motor_idx=2;
    assert(motor_enqueue_command_from_isr(&in));
    tick+=501; motor_set_ros_alive();
    assert(pump_one_command(&out,&status) && status==HAL_TIMEOUT);
    assert(motor_last_failure.reason==MOTOR_REASON_QUEUE_EXPIRED);
    drain(); assert(sent_count[1]==1 && motors[2].target_velocity==-50);
    /* Communication loss still stops once and never resumes old motion. */
    tick+=501; motor_safety_poll(); expect_actual_stop();
    unsigned stopped=sent_count[1];
    motor_safety_poll(); drain(); assert(sent_count[1]==stopped);
    motor_set_ros_alive(); motor_safety_poll(); drain(); assert(sent_count[1]==stopped);
    /* A real motor fault still parks, inhibits motion and is not spammed. */
    motors[1].fault_flag=1; motor_safety_poll(); expect_actual_stop();
    stopped=sent_count[1]; motor_safety_poll(); drain(); assert(sent_count[1]==stopped);
    assert(motor_enqueue_command_from_isr(&in));
    assert(pump_one_command(&out,&status) && status==HAL_ERROR);
    assert(motor_last_failure.reason==MOTOR_REASON_FAULT);
    drain(); assert(sent_count[1]==stopped);
    reset();
    /* Position modes are forwarded without local geometry/reference gating. */
    in.cmd=MOTOR_CMD_SET_POSITION; in.value=3200;
    for(uint8_t mode=0;mode<3;++mode) {
        in.param1=mode;
        assert(motor_enqueue_command_from_isr(&in));
        assert(pump_one_command(&out,&status) && status==HAL_OK);
    }
    puts("PASS ROS-owned braking, immediate single wheel, validation-only rejection, retained link/fault stop and position modes");
}

static void test_reply_priority_and_recovery(void)
{
    CAN_TxHeaderTypeDef telemetry = {.StdId = 0x101U, .DLC = 8U};
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    uint8_t data[8] = {0x12U, 0x34U, 9U};
    CAN_BusDiagnostics_t diag;
    reset();
    for (unsigned i = 0U; i < CAN_TX_QUEUE_LENGTH; i++)
        assert(can_transport_send(&hcan1, &telemetry, data) == HAL_OK);
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    drain();
    assert(sent_count[0] == CAN_TX_QUEUE_LENGTH + 1U);
    assert(sent[0][0].header.StdId == 0x102U);
    assert(memcmp(sent[0][0].data, data, 8U) == 0);
    for (unsigned i = 1U; i < sent_count[0]; i++) assert(sent[0][i].header.StdId == 0x101U);

    reset();
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_TERR0;
    can_transport_service();
    data[1]++;
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK); /* during backoff */
    tick = CAN_ROS_TX_BACKOFF_MS - 1U;
    can_transport_service();
    assert(sent_count[0] == 1U);
    tick++;
    drain();
    assert(sent_count[0] == 3U);
    assert(sent[0][1].data[1] == 0x34U && sent[0][2].data[1] == 0x35U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.reply_pending == 0U && diag.reply_retries == 1U && diag.tx_ok == 2U);

    /* A failed reply is retried for a finite lifetime, not forever. */
    reset();
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    for (tick = 0U; tick < 1000U; tick++) {
        free_mailboxes[0] = 3U;
        can_registers[0].TSR = CAN_TSR_TERR0;
        can_transport_service();
    }
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(sent_count[0] == 5U && diag.reply_expired == 1U && diag.reply_pending == 0U);

    /* Stuck hardware cancellation preserves replies, then recovery retries. */
    reset();
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    tick = CAN_ROS_TX_TIMEOUT_MS;
    can_transport_service();
    assert(aborted[0] == 1U && sent_count[0] == 1U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.reply_pending == 1U && diag.tx_timeouts == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 2U);

    reset();
    for (unsigned i = 0U; i < CAN_REPLY_QUEUE_LENGTH; i++)
        assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_BUSY);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.reply_overflow == 1U && diag.reply_pending == CAN_REPLY_QUEUE_LENGTH);
    can_transport_cancel(&hcan1);
    drain();
    assert(sent_count[0] == 0U);

    reset();
    tick = UINT32_MAX - 99U;
    free_mailboxes[0] = 0U;
    assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    tick += CAN_REPLY_LIFETIME_MS;
    drain();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(sent_count[0] == 0U && diag.reply_expired == 1U);

    /* Exercise the real queue-rejection ACK with telemetry already full. */
    reset();
    Motor_Command_t command = {.cmd = MOTOR_CMD_HEARTBEAT};
    for (unsigned i = 0U; i < MOTOR_CMD_QUEUE_LENGTH; i++)
        assert(motor_enqueue_command_from_isr(&command) == 1U);
    for (unsigned i = 0U; i < CAN_TX_QUEUE_LENGTH; i++)
        assert(can_transport_send(&hcan1, &telemetry, data) == HAL_OK);
    CAN_RxHeaderTypeDef rx = {.StdId = 0x100U, .DLC = 8U};
    CAN1_RxCallback(rx, (uint8_t[]){9U, 255U, 0U, 0U, 0U, 0U, 0U, 0U});
    drain();
    assert(sent[0][0].header.StdId == 0x102U && sent[0][0].data[3] == 1U);
    puts("PASS independent reply capacity, priority, bounded error retry, timeout recovery, expiry and rejection ACK");
}

static void test_can_error_diagnostics(void)
{
    void CAN1_SCE_IRQHandler(void);
    void CAN2_SCE_IRQHandler(void);
    CAN_BusDiagnostics_t diag;
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    reset();
    tick = 42U;
    can_registers[0].MCR = 0x00010054U;
    can_registers[0].BTR = 0x00180005U;
    can_registers[0].ESR = 0x00800033U;
    can_registers[0].MSR = CAN_MSR_ERRI;
    CAN1_SCE_IRQHandler();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.last_error_esr == 0x00800033U && diag.last_error_tick == 42U);
    assert(diag.esr == 0x00800003U && diag.error_events == 1U);
    assert(diag.lec_events[3] == 1U && diag.error_passive_events == 1U && diag.error_warning_events == 1U);
    assert(diag.mcr == 0x00010054U && diag.btr == 0x00180005U);
    can_registers[1].ESR = 0x01000061U;
    can_registers[1].MSR = CAN_MSR_ERRI;
    CAN2_SCE_IRQHandler();
    can_transport_get_diagnostics(&hcan2, &diag);
    assert(diag.last_error_esr == 0x01000061U && diag.error_events == 1U);
    /* HAL software state still LISTENING: BOFF must suppress submissions. */
    can_registers[0].ESR = CAN_ESR_BOFF;
    assert(can_transport_send_reply(&hcan1, &ack, (uint8_t[8]){0}) == HAL_OK);
    drain();
    assert(sent_count[0] == 0U);
    can_registers[0].ESR = 0U;
    drain();
    assert(sent_count[0] == 1U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.tx_ok == 1U && diag.last_error_esr == 0x00800033U);
    CAN_RxHeaderTypeDef rx = {.StdId = 0x321U};
    can_transport_record_rx(&hcan1, &rx);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.rx_frames == 1U && diag.last_rx_id == 0x321U);
    can_registers[0].RF0R = CAN_RF0R_FOVR0;
    can_transport_capture_rx_overrun(&hcan1);
    can_registers[0].RF0R = 0U;
    can_transport_capture_rx_overrun(&hcan1);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.rx_overruns == 1U);
    puts("PASS per-controller ESR capture before HAL clears LEC, latched evidence and bus-off TX gating");
}

#include "can_latest_checks.inc"
#include "can_forward_checks.inc"
#include "motion_execution_checks.inc"

int main(void)
{
    test_ros_owned_parking();
    test_async_motion_outcomes();
    test_button_cache_wait_and_failure();
    test_native_forward_backoff_and_priority();
    test_native_forward_retry_expiry_and_cancel();
    test_native_forward_capacity_and_arbitration();
    test_error_capture_from_receive_irqs();
    test_latest_capacity_and_priority();
    test_latest_in_flight_and_recovery();
    test_latest_ring_and_callbacks();
    test_reply_priority_and_recovery();
    test_can_error_diagnostics();
    test_ros_bus_fault_backoff();
    test_local_pwm_ack_routing();
    test_estop_when_command_queue_full();
    test_pending_survives_transport_full();
    test_response_timeout_waits_for_transmission();
    test_continuous_multi_motor_bursts();
    test_fault_event_handoff_and_irq_mask();
    test_startup_and_lengths();
    test_x_motion_encoding();
    test_mailbox_backpressure();
    test_routing_and_mapping();
    test_sync_timeout_and_stop();
    test_local_sync_ordering_and_ros_gate();
    test_polling_and_faults();
    test_bidirectional_bridge_and_rejection();
    test_aa_bounds_and_length();
    puts("All CAN/control regression tests passed.");
    return 0;
}
