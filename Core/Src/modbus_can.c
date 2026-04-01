#include "modbus_can.h"

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
} Modbus_Pending_t;

static CAN_HandleTypeDef *modbus_can = NULL;
static Modbus_Pending_t pending_req = {0};
static zdt_response_policy_t response_policy = ZDT_RESPONSE_WAIT_ACK;
static uint32_t timeout_drop_count = 0U;

static void clear_pending(void)
{
    (void)memset(&pending_req, 0, sizeof(pending_req));
}

static HAL_StatusTypeDef zdt_send_frames(uint8_t slave, const uint8_t *payload, uint8_t payload_len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t packet_data[8];
    uint8_t packet_idx;
    uint8_t i;
    uint8_t sent;
    uint32_t tx_mailbox;

    if ((modbus_can == NULL) || (payload == NULL) || (payload_len == 0U)) {
        return HAL_ERROR;
    }

    packet_idx = 0U;
    sent = 0U;

    while (sent < payload_len) {
        (void)memset(packet_data, 0, sizeof(packet_data));
        for (i = 0U; (i < 8U) && (sent < payload_len); i++) {
            packet_data[i] = payload[sent++];
        }

        tx_header.StdId = 0U;
        tx_header.ExtId = (((uint32_t)slave) << ZDT_CAN_PACKET_SHIFT) | packet_idx;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8U;
        tx_header.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(modbus_can, &tx_header, packet_data, &tx_mailbox) != HAL_OK) {
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

    need_wait = (type != ZDT_PENDING_NONE) ? 1U : 0U;
    if ((type == ZDT_PENDING_NONE) && (response_policy == ZDT_RESPONSE_WAIT_ACK) && (slave != 0U)) {
        need_wait = 1U;
    }

    if ((pending_req.in_use != 0U) && (need_wait != 0U)) {
        return HAL_BUSY;
    }

    status = zdt_send_frames(slave, payload, payload_len);
    if ((status == HAL_OK) && (need_wait != 0U) && (slave != 0U)) {
        save_pending(slave, payload[0], type, payload, payload_len, speed_cb, position_cb, status_cb);
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

void modbus_can_init(CAN_HandleTypeDef *hcan_bus)
{
    modbus_can = hcan_bus;
    clear_pending();
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
    uint8_t payload[8];
    uint16_t abs_speed;
    uint8_t dir;

    dir = (speed_rpm < 0) ? 0x01U : 0x00U;
    abs_speed = (uint16_t)((speed_rpm < 0) ? (-speed_rpm) : speed_rpm);
    if (abs_speed > 3000U) {
        abs_speed = 3000U;
    }

    payload[0] = ZDT_CMD_SPEED_MODE;
    payload[1] = dir;
    payload[2] = (uint8_t)(abs_speed >> 8);
    payload[3] = (uint8_t)(abs_speed & 0xFFU);
    payload[4] = accel_level;
    payload[5] = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[6] = ZDT_CAN_CHECK_BYTE;
    payload[7] = 0U;

    return send_and_optionally_wait(slave, payload, 7U, ZDT_PENDING_NONE, NULL, NULL, NULL);
}

HAL_StatusTypeDef zdt_motor_set_position(uint8_t slave,
                                         int32_t pulses,
                                         uint16_t speed_rpm,
                                         uint8_t accel_level,
                                         uint8_t mode,
                                         uint8_t sync_flag)
{
    uint8_t payload[13];
    uint32_t abs_pulses;
    uint8_t dir;

    if (speed_rpm > 3000U) {
        speed_rpm = 3000U;
    }

    dir = (pulses < 0) ? 0x01U : 0x00U;
    abs_pulses = (uint32_t)((pulses < 0) ? (-pulses) : pulses);

    payload[0] = ZDT_CMD_POSITION_MODE;
    payload[1] = dir;
    payload[2] = (uint8_t)(speed_rpm >> 8);
    payload[3] = (uint8_t)(speed_rpm & 0xFFU);
    payload[4] = accel_level;
    payload[5] = (uint8_t)(abs_pulses >> 24);
    payload[6] = (uint8_t)(abs_pulses >> 16);
    payload[7] = (uint8_t)(abs_pulses >> 8);
    payload[8] = (uint8_t)(abs_pulses & 0xFFU);
    payload[9] = mode;
    payload[10] = (sync_flag != 0U) ? ZDT_SYNC_CACHE : ZDT_SYNC_IMMEDIATE;
    payload[11] = ZDT_CAN_CHECK_BYTE;
    payload[12] = 0U;

    return send_and_optionally_wait(slave, payload, 12U, ZDT_PENDING_NONE, NULL, NULL, NULL);
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

void modbus_process_response(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8])
{
    uint8_t slave;
    uint8_t packet;
    uint8_t func;

    if ((rxHeader.IDE != CAN_ID_EXT) || (rxHeader.DLC == 0U)) {
        return;
    }

    slave = (uint8_t)((rxHeader.ExtId >> ZDT_CAN_PACKET_SHIFT) & 0xFFU);
    packet = (uint8_t)(rxHeader.ExtId & ZDT_CAN_PACKET_MASK);
    if (packet != 0U) {
        return;
    }

    if (pending_req.in_use == 0U) {
        return;
    }

    if (slave != pending_req.slave) {
        return;
    }

    func = rxData[0];
    if (func != pending_req.function_code) {
        return;
    }

    if ((rxHeader.DLC >= 2U) && ((rxData[1] == 0xE2U) || (rxData[1] == 0xEEU))) {
        clear_pending();
        return;
    }

    switch (pending_req.type) {
        case ZDT_PENDING_READ_SPEED:
            if ((pending_req.speed_cb != NULL) && (rxHeader.DLC >= 4U)) {
                int32_t speed = (int32_t)(((uint16_t)rxData[1] << 8) | rxData[2]);
                pending_req.speed_cb(slave, speed);
            }
            break;

        case ZDT_PENDING_READ_POSITION:
            if ((pending_req.position_cb != NULL) && (rxHeader.DLC >= 7U)) {
                uint32_t pos = ((uint32_t)rxData[2] << 24) |
                               ((uint32_t)rxData[3] << 16) |
                               ((uint32_t)rxData[4] << 8) |
                               ((uint32_t)rxData[5]);
                int32_t signed_pos = (int32_t)pos;
                if (rxData[1] != 0U) {
                    signed_pos = -signed_pos;
                }
                pending_req.position_cb(slave, signed_pos);
            }
            break;

        case ZDT_PENDING_READ_STATUS:
            if ((pending_req.status_cb != NULL) && (rxHeader.DLC >= 3U)) {
                pending_req.status_cb(slave, rxData[1]);
            }
            break;

        case ZDT_PENDING_NONE:
        default:
            break;
    }

    clear_pending();
}

void modbus_timeout_poll(void)
{
    uint32_t now;

    if (pending_req.in_use == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((now - pending_req.last_tick) < 20U) {
        return;
    }

    if (pending_req.retries < 3U) {
        pending_req.retries++;
        pending_req.last_tick = now;
        (void)zdt_send_frames(pending_req.slave, pending_req.tx_data, pending_req.tx_len);
    } else {
        timeout_drop_count++;
        clear_pending();
    }
}

uint32_t modbus_get_timeout_drop_count(void)
{
    return timeout_drop_count;
}
