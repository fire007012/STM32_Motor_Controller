#include "zdt_can_driver.h"

#include <string.h>

typedef enum {
    ZDT_PENDING_NONE = 0,
    ZDT_PENDING_READ_SPEED,
    ZDT_PENDING_READ_POSITION,
    ZDT_PENDING_READ_STATUS
} ZDT_PendingType_t;

typedef struct {
    uint8_t in_use;
    uint8_t slave;
    uint8_t function_code;
    uint8_t tx_len;
    ZDT_PendingType_t type;
    uint8_t retries;
    uint32_t last_tick;
    uint8_t tx_data[24];
    zdt_speed_cb_t speed_cb;
    zdt_position_cb_t position_cb;
    zdt_status_cb_t status_cb;
} ZDT_PendingReq_t;

static CAN_HandleTypeDef *zdt_can = NULL;
static ZDT_PendingReq_t pending_req = {0};
static ZDT_PendingReq_t pending_queue[4] = {0};
static uint8_t pending_head = 0U;
static uint8_t pending_tail = 0U;
static uint8_t pending_count = 0U;

static zdt_response_policy_t response_policy = ZDT_RESPONSE_WAIT_ACK;
static uint32_t timeout_drop_count = 0U;
static uint32_t tx_fail_count = 0U;

static uint32_t zdt_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void zdt_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void clear_pending(void)
{
    (void)memset(&pending_req, 0, sizeof(pending_req));
}

static void pending_queue_reset(void)
{
    (void)memset(pending_queue, 0, sizeof(pending_queue));
    pending_head = 0U;
    pending_tail = 0U;
    pending_count = 0U;
}

static HAL_StatusTypeDef zdt_send_frames(uint8_t slave, const uint8_t *payload, uint8_t payload_len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t packet_data[8];
    uint8_t packet_idx;
    uint8_t i;
    uint8_t sent;
    uint32_t tx_mailbox;

    if ((zdt_can == NULL) || (payload == NULL) || (payload_len == 0U)) {
        return HAL_ERROR;
    }

    /* Single-frame: send as-is */
    if (payload_len <= 8U) {
        (void)memset(packet_data, 0, sizeof(packet_data));
        (void)memcpy(packet_data, payload, payload_len);

        tx_header.StdId = 0U;
        tx_header.ExtId = (((uint32_t)slave) << ZDT_CAN_PACKET_SHIFT) | 0U;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8U;
        tx_header.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(zdt_can, &tx_header, packet_data, &tx_mailbox) != HAL_OK) {
            tx_fail_count++;
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    /* Multi-frame: every frame starts with function code (payload[0]) */
    /* Frame 0: func + payload[1..7] (8 bytes) */
    /* Frame N: func + remaining data bytes */
    uint8_t func = payload[0];
    sent = 1U;
    packet_idx = 0U;

    while (sent < payload_len) {
        (void)memset(packet_data, 0, sizeof(packet_data));
        packet_data[0] = func;
        for (i = 1U; (i < 8U) && (sent < payload_len); i++) {
            packet_data[i] = payload[sent++];
        }

        tx_header.StdId = 0U;
        tx_header.ExtId = (((uint32_t)slave) << ZDT_CAN_PACKET_SHIFT) | packet_idx;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8U;
        tx_header.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(zdt_can, &tx_header, packet_data, &tx_mailbox) != HAL_OK) {
            tx_fail_count++;
            return HAL_ERROR;
        }

        packet_idx++;
    }

    return HAL_OK;
}

static void save_pending(uint8_t slave,
                         uint8_t function_code,
                         ZDT_PendingType_t type,
                         const uint8_t *payload,
                         uint8_t payload_len,
                         zdt_speed_cb_t speed_cb,
                         zdt_position_cb_t position_cb,
                         zdt_status_cb_t status_cb)
{
    pending_req.in_use = 1U;
    pending_req.slave = slave;
    pending_req.function_code = function_code;
    pending_req.type = type;
    pending_req.tx_len = payload_len;
    pending_req.retries = 0U;
    pending_req.last_tick = HAL_GetTick();
    pending_req.speed_cb = speed_cb;
    pending_req.position_cb = position_cb;
    pending_req.status_cb = status_cb;

    if ((payload != NULL) && (payload_len <= sizeof(pending_req.tx_data))) {
        (void)memcpy(pending_req.tx_data, payload, payload_len);
    }
}

static uint8_t enqueue_pending(uint8_t slave,
                               uint8_t function_code,
                               ZDT_PendingType_t type,
                               const uint8_t *payload,
                               uint8_t payload_len,
                               zdt_speed_cb_t speed_cb,
                               zdt_position_cb_t position_cb,
                               zdt_status_cb_t status_cb)
{
    ZDT_PendingReq_t *slot;
    uint8_t queue_size = (uint8_t)(sizeof(pending_queue) / sizeof(pending_queue[0]));
    uint32_t primask;

    if ((payload == NULL) || (payload_len == 0U) || (payload_len > sizeof(pending_queue[0].tx_data))) {
        return 0U;
    }

    primask = zdt_enter_critical();
    if (pending_count >= queue_size) {
        zdt_exit_critical(primask);
        return 0U;
    }

    slot = &pending_queue[pending_tail];
    (void)memset(slot, 0, sizeof(*slot));
    slot->in_use = 1U;
    slot->slave = slave;
    slot->function_code = function_code;
    slot->type = type;
    slot->tx_len = payload_len;
    slot->speed_cb = speed_cb;
    slot->position_cb = position_cb;
    slot->status_cb = status_cb;
    (void)memcpy(slot->tx_data, payload, payload_len);

    pending_tail = (uint8_t)((pending_tail + 1U) % queue_size);
    pending_count++;
    zdt_exit_critical(primask);
    return 1U;
}

static uint8_t dequeue_pending(ZDT_PendingReq_t *out)
{
    ZDT_PendingReq_t *slot;
    uint8_t queue_size = (uint8_t)(sizeof(pending_queue) / sizeof(pending_queue[0]));
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = zdt_enter_critical();
    if (pending_count == 0U) {
        zdt_exit_critical(primask);
        return 0U;
    }

    slot = &pending_queue[pending_head];
    *out = *slot;
    (void)memset(slot, 0, sizeof(*slot));

    pending_head = (uint8_t)((pending_head + 1U) % queue_size);
    pending_count--;
    zdt_exit_critical(primask);
    return 1U;
}

static void dispatch_next_pending(void)
{
    ZDT_PendingReq_t req;
    uint32_t primask;

    primask = zdt_enter_critical();
    if ((pending_req.in_use != 0U) || (pending_count == 0U)) {
        zdt_exit_critical(primask);
        return;
    }
    zdt_exit_critical(primask);

    if (dequeue_pending(&req) == 0U) {
        return;
    }

    primask = zdt_enter_critical();
    pending_req = req;
    pending_req.retries = 0U;
    pending_req.last_tick = HAL_GetTick();
    pending_req.in_use = 1U;
    zdt_exit_critical(primask);

    if (zdt_send_frames(req.slave, req.tx_data, req.tx_len) != HAL_OK) {
        timeout_drop_count++;
        primask = zdt_enter_critical();
        clear_pending();
        zdt_exit_critical(primask);
        return;
    }
}

static HAL_StatusTypeDef send_and_optionally_wait(uint8_t slave,
                                                  const uint8_t *payload,
                                                  uint8_t payload_len,
                                                  ZDT_PendingType_t type,
                                                  zdt_speed_cb_t speed_cb,
                                                  zdt_position_cb_t position_cb,
                                                  zdt_status_cb_t status_cb)
{
    HAL_StatusTypeDef status;
    uint8_t need_wait;
    uint32_t primask;
    uint8_t has_pending;

    need_wait = (type != ZDT_PENDING_NONE) ? 1U : 0U;
    if ((type == ZDT_PENDING_NONE) && (response_policy == ZDT_RESPONSE_WAIT_ACK) && (slave != 0U)) {
        need_wait = 1U;
    }

    primask = zdt_enter_critical();
    has_pending = pending_req.in_use;
    zdt_exit_critical(primask);

    if ((need_wait != 0U) && (slave != 0U) && (has_pending != 0U)) {
        if (enqueue_pending(slave, payload[0], type, payload, payload_len, speed_cb, position_cb, status_cb) == 0U) {
            return HAL_BUSY;
        }
        return HAL_OK;
    }

    status = HAL_OK;
    if ((status == HAL_OK) && (need_wait != 0U) && (slave != 0U)) {
        primask = zdt_enter_critical();
        save_pending(slave, payload[0], type, payload, payload_len, speed_cb, position_cb, status_cb);
        zdt_exit_critical(primask);
    }

    status = zdt_send_frames(slave, payload, payload_len);
    if ((status != HAL_OK) && (need_wait != 0U) && (slave != 0U)) {
        primask = zdt_enter_critical();
        clear_pending();
        zdt_exit_critical(primask);
    }

    return status;
}

void zdt_set_response_policy(zdt_response_policy_t policy)
{
    response_policy = policy;
}

zdt_response_policy_t zdt_get_response_policy(void)
{
    return response_policy;
}

void zdt_can_driver_init(CAN_HandleTypeDef *hcan_bus)
{
    zdt_can = hcan_bus;
    clear_pending();
    pending_queue_reset();
    tx_fail_count = 0U;
}

HAL_StatusTypeDef zdt_motor_enable(uint8_t slave, uint8_t enable, uint8_t sync_flag)
{
    uint8_t payload[5];

    payload[0] = ZDT_CMD_MOTOR_ENABLE;
    payload[1] = 0xABU;
    payload[2] = (enable != 0U) ? 0x01U : 0x00U;
    payload[3] = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[4] = ZDT_CAN_CHECK_BYTE;

    return send_and_optionally_wait(slave, payload, (uint8_t)sizeof(payload), ZDT_PENDING_NONE, NULL, NULL, NULL);
}

HAL_StatusTypeDef zdt_motor_set_speed(uint8_t slave, int32_t speed_rpm, uint8_t accel_level, uint8_t sync_flag)
{
    /* X firmware F6: dir(1) + accel(2B RPM/s) + speed(2B ×0.1RPM) + sync(1) + chk(1) = 8 bytes */
    uint8_t payload[8];
    uint16_t x_speed;
    uint8_t dir;

    dir = (speed_rpm < 0) ? 0x01U : 0x00U;
    x_speed = (uint16_t)((speed_rpm < 0) ? (-speed_rpm) : speed_rpm);
    if (x_speed > 3000U) { x_speed = 3000U; }
    x_speed = (uint16_t)(x_speed * 10U);

    payload[0] = 0xF6U;
    payload[1] = dir;
    payload[2] = (uint8_t)((uint16_t)accel_level >> 8);
    payload[3] = (uint8_t)((uint16_t)accel_level & 0xFFU);
    payload[4] = (uint8_t)(x_speed >> 8);
    payload[5] = (uint8_t)(x_speed & 0xFFU);
    payload[6] = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[7] = ZDT_CAN_CHECK_BYTE;

    return send_and_optionally_wait(slave, payload, 8U, ZDT_PENDING_NONE, NULL, NULL, NULL);
}

HAL_StatusTypeDef zdt_motor_set_position(uint8_t slave,
                                         int32_t pulses,
                                         uint16_t speed_rpm,
                                         uint8_t accel_level,
                                         uint8_t mode,
                                         uint8_t sync_flag)
{
    /* X firmware FB: dir(1)+speed(2B ×0.1RPM)+pos(4B ×0.1°)+mode(1)+sync(1)+chk(1)=11 bytes */
    uint8_t payload[12];
    uint32_t x_pos;
    uint16_t x_speed;
    uint8_t dir;

    (void)accel_level; /* X firmware FB does not use accel field */

    /* pulses → degrees×10: 3200pulse/360° → deg×10 = pulses * 9 / 8 */
    x_pos = (pulses < 0) ? ((uint32_t)(-pulses) * 9U / 8U)
                         : ((uint32_t)pulses * 9U / 8U);
    dir = (pulses < 0) ? 0x01U : 0x00U;

    if (speed_rpm > 3000U) { speed_rpm = 3000U; }
    x_speed = speed_rpm * 10U;

    payload[0]  = 0xFBU;
    payload[1]  = dir;
    payload[2]  = (uint8_t)(x_speed >> 8);
    payload[3]  = (uint8_t)(x_speed & 0xFFU);
    payload[4]  = (uint8_t)(x_pos >> 24);
    payload[5]  = (uint8_t)(x_pos >> 16);
    payload[6]  = (uint8_t)(x_pos >> 8);
    payload[7]  = (uint8_t)(x_pos & 0xFFU);
    payload[8]  = mode;
    payload[9]  = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[10] = ZDT_CAN_CHECK_BYTE;

    return send_and_optionally_wait(slave, payload, 11U, ZDT_PENDING_NONE, NULL, NULL, NULL);
}

HAL_StatusTypeDef zdt_motor_stop(uint8_t slave, uint8_t sync_flag)
{
    uint8_t payload[5];

    payload[0] = ZDT_CMD_EMERGENCY_STOP;
    payload[1] = 0x98U;
    payload[2] = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[3] = ZDT_CAN_CHECK_BYTE;
    payload[4] = 0U;

    return send_and_optionally_wait(slave, payload, 4U, ZDT_PENDING_NONE, NULL, NULL, NULL);
}

HAL_StatusTypeDef zdt_trigger_sync_start(void)
{
    uint8_t payload[3];

    payload[0] = ZDT_CMD_SYNC_TRIGGER;
    payload[1] = 0x66U;
    payload[2] = ZDT_CAN_CHECK_BYTE;

    return zdt_send_frames(0U, payload, (uint8_t)sizeof(payload));
}

HAL_StatusTypeDef zdt_send_multi_motor_command(const uint8_t *cmd_stream, uint16_t stream_len)
{
    uint8_t payload[96];
    uint16_t total_len;

    if ((cmd_stream == NULL) || (stream_len == 0U)) {
        return HAL_ERROR;
    }

    total_len = (uint16_t)(stream_len + 4U);
    if (total_len > (uint16_t)sizeof(payload)) {
        return HAL_ERROR;
    }

    payload[0] = 0xAAU;
    payload[1] = (uint8_t)(stream_len >> 8);
    payload[2] = (uint8_t)(stream_len & 0xFFU);
    (void)memcpy(&payload[3], cmd_stream, stream_len);
    payload[(uint16_t)(3U + stream_len)] = ZDT_CAN_CHECK_BYTE;

    return zdt_send_frames(0U, payload, (uint8_t)total_len);
}

HAL_StatusTypeDef zdt_read_realtime_speed(uint8_t slave, zdt_speed_cb_t callback)
{
    uint8_t payload[2];

    payload[0] = ZDT_CMD_READ_REALTIME_SPEED;
    payload[1] = ZDT_CAN_CHECK_BYTE;
    return send_and_optionally_wait(slave, payload, (uint8_t)sizeof(payload), ZDT_PENDING_READ_SPEED, callback, NULL, NULL);
}

HAL_StatusTypeDef zdt_read_realtime_position(uint8_t slave, zdt_position_cb_t callback)
{
    uint8_t payload[2];

    payload[0] = ZDT_CMD_READ_REALTIME_POSITION;
    payload[1] = ZDT_CAN_CHECK_BYTE;
    return send_and_optionally_wait(slave, payload, (uint8_t)sizeof(payload), ZDT_PENDING_READ_POSITION, NULL, callback, NULL);
}

HAL_StatusTypeDef zdt_read_motor_status(uint8_t slave, zdt_status_cb_t callback)
{
    uint8_t payload[2];

    payload[0] = ZDT_CMD_READ_MOTOR_STATUS;
    payload[1] = ZDT_CAN_CHECK_BYTE;
    return send_and_optionally_wait(slave, payload, (uint8_t)sizeof(payload), ZDT_PENDING_READ_STATUS, NULL, NULL, callback);
}

void zdt_can_driver_process_response(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8])
{
    uint8_t slave;
    uint8_t packet;
    uint8_t func;
    ZDT_PendingReq_t active_req;
    uint32_t primask;

    if ((rxHeader.IDE != CAN_ID_EXT) || (rxHeader.DLC == 0U)) {
        return;
    }

    slave = (uint8_t)((rxHeader.ExtId >> ZDT_CAN_PACKET_SHIFT) & 0xFFU);
    packet = (uint8_t)(rxHeader.ExtId & ZDT_CAN_PACKET_MASK);
    if (packet != 0U) {
        return;
    }

    primask = zdt_enter_critical();
    active_req = pending_req;
    zdt_exit_critical(primask);

    if (active_req.in_use == 0U) {
        return;
    }

    if (slave != active_req.slave) {
        return;
    }

    func = rxData[0];
    if (func != active_req.function_code) {
        return;
    }

    if ((rxHeader.DLC >= 2U) && ((rxData[1] == 0xE2U) || (rxData[1] == 0xEEU))) {
        clear_pending();
        dispatch_next_pending();
        return;
    }

    switch (active_req.type) {
        case ZDT_PENDING_READ_SPEED: {
            if ((active_req.speed_cb != NULL) && (rxHeader.DLC >= 4U)) {
                int32_t speed_01rpm = (int32_t)(((uint16_t)rxData[2] << 8) | rxData[3]);
                if (rxData[1] != 0U) {
                    speed_01rpm = -speed_01rpm;
                }
                active_req.speed_cb(slave, speed_01rpm);
            }
            break;
        }

        case ZDT_PENDING_READ_POSITION:
            if ((active_req.position_cb != NULL) && (rxHeader.DLC >= 7U)) {
                uint32_t pos = ((uint32_t)rxData[2] << 24) |
                               ((uint32_t)rxData[3] << 16) |
                               ((uint32_t)rxData[4] << 8) |
                               ((uint32_t)rxData[5]);
                int32_t signed_pos = (int32_t)pos;
                if (rxData[1] != 0U) {
                    signed_pos = -signed_pos;
                }
                active_req.position_cb(slave, signed_pos);
            }
            break;

        case ZDT_PENDING_READ_STATUS: {
            if ((active_req.status_cb != NULL) && (rxHeader.DLC >= 3U)) {
                uint8_t status_flags = rxData[1];
                if ((rxHeader.DLC >= 4U) && (rxData[1] == 0x00U)) {
                    status_flags = rxData[2];
                }
                active_req.status_cb(slave, status_flags);
            }
            break;
        }

        case ZDT_PENDING_NONE:
        default:
            break;
    }

    clear_pending();
    dispatch_next_pending();
}

void zdt_can_driver_timeout_poll(void)
{
    uint32_t now;
    uint32_t primask;
    ZDT_PendingReq_t active_req;

    primask = zdt_enter_critical();
    active_req = pending_req;
    zdt_exit_critical(primask);

    if (active_req.in_use == 0U) {
        dispatch_next_pending();
        return;
    }

    now = HAL_GetTick();
    if ((now - active_req.last_tick) < 20U) {
        return;
    }

    if (active_req.retries < 3U) {
        primask = zdt_enter_critical();
        pending_req.retries++;
        pending_req.last_tick = now;
        active_req = pending_req;
        zdt_exit_critical(primask);
        (void)zdt_send_frames(active_req.slave, active_req.tx_data, active_req.tx_len);
    } else {
        timeout_drop_count++;
        primask = zdt_enter_critical();
        clear_pending();
        zdt_exit_critical(primask);
        dispatch_next_pending();
    }
}

uint32_t zdt_can_driver_get_timeout_drop_count(void)
{
    return timeout_drop_count;
}

uint32_t zdt_can_driver_get_tx_fail_count(void)
{
    return tx_fail_count;
}
