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
#include "chassis_collision.h"
#include <math.h>

CAN_HandleTypeDef hcan1 = {0U}, hcan2 = {1U};
TIM_HandleTypeDef htim6;
uint32_t test_primask;
static uint32_t tick;
static unsigned free_mailboxes[2];
static unsigned fail_next[2];
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
    assert(free_mailboxes[i] != 0U);
    assert(sent_count[i] < 512U);
    free_mailboxes[i]--;
    sent[i][sent_count[i]].header = *header;
    memcpy(sent[i][sent_count[i]++].data, data, 8U);
    *mailbox = 1U << (2U - free_mailboxes[i]);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *bus, uint32_t mailboxes)
{
    assert(mailboxes == 7U);
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
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *bus) { (void)bus; }
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
    memset(aborted, 0, sizeof(aborted));
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
    for (i = 0U; i < 32U; i++) {
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
    assert(sent_count[1] == 1U); /* canceled backlog, actual broadcast stop */
    expect_frame(1, 0, 0, (uint8_t[]){0xFE, 0x98, 0, 0x6B}, 4);
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
        assert(motor_process_next_command(&executed, &result) == 1U && result == HAL_OK);
        assert(executed.cmd == MOTOR_CMD_SET_POSITION);
        assert(executed.source == MOTOR_CMD_SOURCE_REMOTE);
    }
    assert(motor_process_next_command(&executed, &result) == 1U);
    assert(executed.cmd == MOTOR_CMD_SYNC_START && result == HAL_OK);
    assert(executed.source == MOTOR_CMD_SOURCE_LOCAL_BUTTON);
    assert(executed.seq == 0x1234U);

    assert(motor_request_local_sync_start(0x1235U) == 1U);
    assert(motor_stop_all() == HAL_OK);
    assert(motor_process_next_command(&executed, &result) == 0U);
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
        } else if (functions[i]==0x36U) {
            reply(addresses[i],(uint8_t[]){0x36,0,0,0,0x0E,0x10,0x6B},7);
            assert(motors[0].position_feedback == 3600);
        } else if(i==1U) {
            reply(2,(uint8_t[]){0x3A,0x83,0x6B},3);
            reply(1,(uint8_t[]){0x3A,0x83,0},3);
            assert(zdt_can_driver_is_busy());
            reply(1,(uint8_t[]){0x3A,0x83,0x6B},3);
            assert(motors[0].status_word==0x83 && !motors[0].fault_flag);
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
        unsigned processed = 0U;
        while (motor_process_next_command(&executed, &result) != 0U) {
            assert(result == HAL_OK && test_primask == 0U);
            processed++;
            drain();
        }
        assert(processed == MOTOR_COUNT && command_count == 0U);
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


static void range_sample(uint8_t id, uint16_t raw, uint8_t status)
{
    distance_sensor_sample_t sample = {.sensor_id=id, .distance_mm=raw,
        .status=status, .valid=(status & 1U) != 0U, .timestamp_ms=tick};
    motor_update_distance(&sample);
}

/* Complete synthetic RPM group: this fixture is NOT the real wheel geometry. */
static HAL_StatusTypeDef group(int32_t a, int32_t b, int32_t c, int32_t d)
{
    int32_t rpm[4] = {a,b,c,d};
    for (uint8_t i=0U; i<4U; ++i)
        assert(motor_set_velocity_ex(i, rpm[i], 50U, 1U) == HAL_OK);
    return motor_trigger_sync_motion();
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

static void test_collision_thresholds_and_directions(void)
{
    assert(fabsf(CHASSIS_MOTOR_RPM_TO_WHEEL_MPS-0.0078539816f)<0.0000001f);
    assert(fabsf(chassis_collision_stop_mm(0.20f)-160.0f)<0.001f);
    assert(fabsf(chassis_collision_stop_mm(0.25f)-181.25f)<0.001f);
    const uint8_t ids[4]={0U,1U,2U,5U};
    const int signs[4][4]={{1,1,1,1},{-1,1,1,-1},{1,-1,-1,1},{-1,-1,-1,-1}};
    for (unsigned speed=200U; speed<=250U; speed+=50U) {
        unsigned threshold = speed == 200U ? 160U : 181U;
        for (unsigned dir=0U; dir<4U; ++dir) {
            reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
            range_sample(ids[dir],(uint16_t)(threshold+(dir?50U:0U)+1U),1U);
            assert(group(signs[dir][0]*(int)speed,signs[dir][1]*(int)speed,
                         signs[dir][2]*(int)speed,signs[dir][3]*(int)speed)==HAL_OK);
            drain();
            range_sample(ids[dir],(uint16_t)(threshold+(dir?50U:0U)),1U);
            motor_safety_poll(); expect_actual_stop();
        }
    }
    for (unsigned dir=1U; dir<4U; ++dir) {
        reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
        range_sample(ids[dir],50U,1U); /* valid zero clearance, not missing */
        assert(group(signs[dir][0]*50,signs[dir][1]*50,
                     signs[dir][2]*50,signs[dir][3]*50)==HAL_ERROR);
        expect_actual_stop();
    }
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(5U,50U,1U); /* rear obstacle does not prevent forward departure */
    assert(group(200,200,200,200)==HAL_OK);
    range_sample(0U,100U,1U); motor_safety_poll(); expect_actual_stop();
    assert(group(-200,200,200,-200)==HAL_OK); /* new safe sideways request */
    drain();
    for (unsigned i=0U; i<4U; ++i) assert(motors[i].target_velocity != 0);
    puts("PASS acceptance 2-5,7: directional 160/210 and 181.25/231.25 mm, zero clearance, actual stop and safe lateral escape");
}

static void test_collision_missing_rotation_calibration_hysteresis(void)
{
    const uint8_t ids[4]={0U,1U,2U,5U};
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    for(unsigned i=0U;i<4U;++i) range_sample(ids[i],0xFFFFU,0x10U);
    assert(group(200,200,200,200)==HAL_OK); drain();
    motor_safety_poll(); drain(); assert(motors[0].target_velocity==2000);
    range_sample(3U,50U,1U); range_sample(4U,50U,1U);
    motor_safety_poll(); assert(motors[0].target_velocity==2000);
    for(unsigned dir=0U;dir<4U;++dir) {
        reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
        range_sample(ids[dir],(uint16_t)(dir?140U:90U),1U);
        assert(group(-20,20,-20,20)==HAL_ERROR); expect_actual_stop();
        range_sample(ids[dir],(uint16_t)(dir?180U:130U),1U);
        assert(group(-20,20,-20,20)==HAL_ERROR); /* release strictly >100+30 */
        range_sample(ids[dir],(uint16_t)(dir?181U:131U),1U);
        assert(group(-20,20,-20,20)==HAL_OK);
    }
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0U,250U,1U); assert(group(-20,20,-20,20)==HAL_OK);
    range_sample(1U,140U,1U);
    assert(group(180,220,180,220)==HAL_ERROR); /* vx=.2 + turn, left rotation clearance=90 */
    range_sample(1U,0xFFFFU,4U);
    for(unsigned mm=270U;mm>=250U;--mm) {
        range_sample(0U,(uint16_t)mm,1U);
        assert(group(50,50,50,50)==HAL_OK); drain();
    }
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0U,160U,1U); assert(group(200,200,200,200)==HAL_ERROR);
    range_sample(0U,190U,1U); assert(group(200,200,200,200)==HAL_ERROR);
    range_sample(0U,191U,1U); assert(group(200,200,200,200)==HAL_OK);
    drain();
    range_sample(0U,100U,1U); motor_safety_poll(); expect_actual_stop();
    range_sample(0U,0xFFFFU,4U); assert(group(200,200,200,200)==HAL_OK);
    drain();
    range_sample(0U,100U,1U); motor_safety_poll(); expect_actual_stop();
    tick+=CHASSIS_RANGE_FRESH_MS+1U;
    assert(group(200,200,200,200)==HAL_OK); /* expired distance stops blocking */
    puts("PASS acceptance 1,6,8-11: missing/stale ranges, arm exclusion, circle turning, 250 mm calibration and 30 mm hysteresis");
}

static void test_motion_lifetime_and_atomic_sync(void)
{
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0U,100U,1U);
    /* First partial group could look like unsafe forward/rotation. It must
     * remain cached until the full SAFE lateral vector has arrived. */
    assert(group(-200,200,200,-200)==HAL_OK); drain();
    unsigned start_count=sent_count[1];
    assert(motor_set_velocity_ex(0U,200,50U,1U)==HAL_OK);
    assert(motor_trigger_sync_motion()==HAL_ERROR); expect_actual_stop();
    assert(sent_count[1]==start_count+1U); /* no partial trigger escaped */

    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    assert(group(200,200,200,200)==HAL_OK); drain();
    tick=501U; motor_set_ros_alive(); motor_safety_poll(); expect_actual_stop();
    range_sample(0U,1000U,1U); motor_safety_poll(); drain();
    assert(motors[0].target_velocity==0); /* no resurrection after timeout */
    assert(group(200,200,200,200)==HAL_OK); drain();
    tick+=501U; motor_safety_poll(); expect_actual_stop(); /* heartbeat loss */
    motor_set_ros_alive(); motors[2].fault_flag=1U;
    assert(group(200,200,200,200)==HAL_ERROR); expect_actual_stop();

    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    Motor_Command_t cmd={.cmd=MOTOR_CMD_SET_VELOCITY,.motor_idx=0U,.value=200};
    Motor_Command_t out; HAL_StatusTypeDef result;
    assert(motor_enqueue_command_from_isr(&cmd));
    tick=501U; motor_set_ros_alive();
    assert(motor_process_next_command(&out,&result) && result==HAL_TIMEOUT);
    expect_actual_stop();
    for(uint8_t i=0U;i<4U;++i) assert(motor_set_velocity_ex(i,200,50,1)==HAL_OK);
    tick+=501U; motor_set_ros_alive();
    assert(motor_trigger_sync_motion()==HAL_ERROR); expect_actual_stop();

    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    cmd.motor_idx=0xFFU;
    assert(motor_enqueue_command_from_isr(&cmd));
    tick=400U; motor_set_ros_alive();
    assert(motor_process_next_command(&out,&result) && result==HAL_OK);
    drain();
    tick=501U; motor_set_ros_alive(); motor_safety_poll(); expect_actual_stop();
    /* Receiving at t=0 and applying at t=400 must still expire at t>500. */

    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0U,160U,1U);
    for (uint8_t i=0U;i<4U;++i) {
        motor_set_speed_profile(i,200U,50U);
        assert(motor_set_position_ex(i,3200,50,2,1)==HAL_OK);
        motor_set_speed_profile(i,50U,50U);
    }
    /* The cached position was sent at 200 RPM. A later profile must not
     * make the collision calculation pretend that it was sent at 50 RPM. */
    assert(motor_trigger_sync_motion()==HAL_ERROR); expect_actual_stop();
    puts("PASS fresh full sync groups, queue/cache/motion deadlines, heartbeat and severe motor fault");
}

static void test_measured_body_speed_and_no_raw_bypass(void)
{
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    /* Feed actual signed 0.1 RPM replies through the production decoder.
     * All four wheels at +250 RPM -> vx=.25 in the synthetic fixture. */
    uint8_t got=0U;
    for(unsigned step=0U; step<40U && got!=15U; ++step) {
        motor_update_status(1U); drain();
        CAN_TxFrame_t *frame=&sent[1][sent_count[1]-1U];
        uint8_t address=(uint8_t)(frame->header.ExtId>>8);
        if(frame->data[0]==0x35U) {
            reply(address,(uint8_t[]){0x35,0,0x09,0xC4,0x6B},5);
            got|=(uint8_t)(1U<<(address-1U));
        } else if(frame->data[0]==0x36U) reply(address,(uint8_t[]){0x36,0,0,0,0,0,0x6B},7);
        else reply(address,(uint8_t[]){0x3A,0x83,0x6B},3);
        tick += 20U;
    }
    assert(got==15U);
    range_sample(0U,170U,1U);
    assert(group(50,50,50,50)==HAL_ERROR); expect_actual_stop(); /* measured .25 overrides request .05 */
    tick+=CHASSIS_FEEDBACK_FRESH_MS+1U;
    range_sample(0U,170U,1U);
    /* First clear the previous direction's held threshold via missing range. */
    range_sample(0U,0xFFFFU,4U); assert(group(50,50,50,50)==HAL_OK); drain();
    range_sample(0U,170U,1U); motor_safety_poll(); assert(motors[0].target_velocity==500);
    CAN_RxHeaderTypeDef hdr={.IDE=CAN_ID_EXT,.ExtId=0x100,.DLC=8};
    CAN1_RxCallback(hdr,(uint8_t[]){0xF6,0,0,50,3,0xE8,0,0x6B});
    unsigned count=sent_count[1]; drain(); assert(sent_count[1]==count);
    puts("PASS 0.1 RPM feedback conversion, freshness, measured braking speed and raw motion bypass rejection");
}

int main(void)
{
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
    test_collision_thresholds_and_directions();
    test_collision_missing_rotation_calibration_hysteresis();
    test_motion_lifetime_and_atomic_sync();
    test_measured_body_speed_and_no_raw_bypass();
    puts("All CAN/control regression tests passed.");
    return 0;
}
