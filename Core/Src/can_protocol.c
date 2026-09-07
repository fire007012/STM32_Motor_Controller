#include "can_protocol.h"

#include "motor_control.h"

extern CAN_HandleTypeDef hcan2;

static int32_t parse_int32_le(const uint8_t data[8])
{
    uint32_t v;

    v = ((uint32_t)data[2]) |
        ((uint32_t)data[3] << 8) |
        ((uint32_t)data[4] << 16) |
        ((uint32_t)data[5] << 24);
    return (int32_t)v;
}

static void forward_can1_ext_to_can2(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8])
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t txMailbox;

    if (rxHeader == NULL) {
        return;
    }

    txHeader.StdId = 0U;
    txHeader.ExtId = rxHeader->ExtId;
    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = rxHeader->RTR;
    txHeader.DLC = rxHeader->DLC;
    txHeader.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(&hcan2, &txHeader, (uint8_t *)rxData, &txMailbox);
}

static void chassis_speed_to_y42(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8])
{
    /* Parse: [0x01, motor_idx, rpm(int32 LE), checksum] */
    int32_t rpm;
    uint8_t motor_idx = rxData[1];
    uint8_t y42_addr;
    uint16_t x_speed;
    uint8_t dir;
    uint8_t payload[8];
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t txMailbox;

    /* decode int32 LE from bytes 2..5 */
    rpm = (int32_t)( ((uint32_t)rxData[2])       |
                     ((uint32_t)rxData[3] << 8)  |
                     ((uint32_t)rxData[4] << 16) |
                     ((uint32_t)rxData[5] << 24) );

    /* motor_idx -> Y42 address: 0..3→1..4, 0xFF→0(broadcast) */
    y42_addr = (motor_idx == 0xFFU) ? 0U : (uint8_t)(motor_idx + 1U);
    if (y42_addr > 4U && motor_idx != 0xFFU) { return; }

    dir  = (rpm < 0) ? 0x01U : 0x00U;
    x_speed = (uint16_t)((rpm < 0) ? (-rpm) : rpm);
    if (x_speed > 3000U) { x_speed = 3000U; }
    x_speed = (uint16_t)(x_speed * 10U);

    /* Y42 X firmware F6: dir(1) + accel(2B) + speed_x10(2B) + sync(1) + chk(1) */
    payload[0] = 0xF6U;
    payload[1] = dir;
    payload[2] = 0x00U;  payload[3] = 0x32U;    /* accel = 50 RPM/s */
    payload[4] = (uint8_t)(x_speed >> 8);
    payload[5] = (uint8_t)(x_speed & 0xFFU);
    payload[6] = 0x00U;                           /* sync = immediate */
    payload[7] = 0x6BU;                           /* checksum */

    txHeader.ExtId = ((uint32_t)y42_addr << 8U) | 0U;
    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8U;
    txHeader.TransmitGlobalTime = DISABLE;
    (void)HAL_CAN_AddTxMessage(&hcan2, &txHeader, payload, &txMailbox);
}

void CAN1_RxCallback(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8])
{
    Motor_Command_t cmd;
    uint8_t enqueue_ok;

    motor_set_ros_alive();

    if (rxHeader.IDE == CAN_ID_EXT) {
        if (((rxHeader.ExtId & 0x1FFFFFFFU) == 0x700U) && (rxHeader.DLC >= 8U) && (rxData[0] == 0x20U)) {
            Servo_HandleCanCommand(rxData);
            return;
        }
        /* chassis speed cmd (0x201, data[0]=0x01) -> translate to Y42 F6 */
        if ((rxHeader.ExtId & 0x1FFFFFFFU) == 0x201U && rxData[0] == 0x01U) {
            chassis_speed_to_y42(&rxHeader, rxData);
            return;
        }
        /* other extended frames (arm FB etc.) -> transparent forward */
        forward_can1_ext_to_can2(&rxHeader, rxData);
        return;
    }

    if ((rxHeader.IDE != CAN_ID_STD) || (rxHeader.StdId != ROS_CAN_CMD_ID) || (rxHeader.DLC < 8U)) {
        return;
    }

    cmd.cmd = rxData[0];
    cmd.motor_idx = rxData[1];
    cmd.value = parse_int32_le(rxData);
    cmd.param0 = rxData[6];
    cmd.param1 = rxData[7];
    cmd.seq = motor_allocate_cmd_seq();

    if ((cmd.cmd != MOTOR_CMD_SYNC_START) &&
        (cmd.cmd != MOTOR_CMD_SET_RESPONSE_POLICY) &&
        (cmd.cmd != MOTOR_CMD_SET_REPORT_MASK) &&
        (cmd.cmd != MOTOR_CMD_SET_ADDRESS_MAP) &&
        (cmd.cmd != MOTOR_CMD_HEARTBEAT) &&
        (cmd.motor_idx >= MOTOR_COUNT) &&
        (cmd.motor_idx != 0xFFU)) {
        return;
    }

    if ((cmd.cmd == MOTOR_CMD_HEARTBEAT) || (cmd.cmd == MOTOR_CMD_SET_VELOCITY) || (cmd.cmd == MOTOR_CMD_SET_POSITION)) {
        motor_set_ros_alive();
    }

    enqueue_ok = motor_enqueue_command_from_isr(&cmd);
    motor_record_rx_result(enqueue_ok);
}
