#include "can_protocol.h"

#include "can_transport.h"
#include "motor_control.h"
#include "arm_servo_protocol.h"

extern uint8_t ArmServo_HandleCanCommand(const uint8_t data[8]);

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static int32_t parse_int32_le(const uint8_t data[8])
{
    uint32_t value = ((uint32_t)data[2]) |
                     ((uint32_t)data[3] << 8) |
                     ((uint32_t)data[4] << 16) |
                     ((uint32_t)data[5] << 24);
    return (int32_t)value;
}

static void enqueue_command(Motor_Command_t *cmd)
{
    uint8_t accepted;
    cmd->seq = motor_allocate_cmd_seq();
    accepted = motor_enqueue_command_from_isr(cmd);
    motor_record_rx_result(accepted);
    if (accepted != 0U) {
        motor_set_ros_alive();
    } else {
        CAN_TxHeaderTypeDef header = {0};
        uint8_t data[8] = {(uint8_t)(cmd->seq >> 8), (uint8_t)cmd->seq, cmd->cmd, 1U, 0U, 0U, 0U, 0U};
        header.StdId = ROS_CAN_ACK_ID;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8U;
        (void)can_transport_send(&hcan1, &header, data);
    }
}

static void send_servo_ack(uint32_t id, uint8_t code, const uint8_t request[8], uint8_t result)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {code, request[6], result, request[0], request[1], 0U, 0U, 0U};
    uint8_t i;
    header.ExtId = id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    for (i = 0U; i < 7U; ++i) data[7] = (uint8_t)(data[7] + data[i]);
    (void)can_transport_send(&hcan1, &header, data);
}

void CAN1_RxCallback(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8])
{
    Motor_Command_t cmd = {0};
    uint32_t id;

    if ((rxData == NULL) || (rxHeader.RTR != CAN_RTR_DATA) ||
        (rxHeader.DLC == 0U) || (rxHeader.DLC > 8U)) {
        return;
    }
    id = (rxHeader.IDE == CAN_ID_EXT) ? rxHeader.ExtId : rxHeader.StdId;

    /* Local PWM commands must never be forwarded to the old Y42 bus. */
    if ((rxHeader.IDE == CAN_ID_EXT) && (id == ARM_SERVO_CAN_ID)) {
        if (rxHeader.DLC == 8U) {
            uint8_t result = ArmServo_HandleCanCommand(rxData);
            if (result == 0U) {
                if (rxData[0] == ARM_SERVO_CMD_RELEASE) Servo_ReleaseLocalControl();
                else Servo_AcquireRemoteControl();
            }
            send_servo_ack(ARM_SERVO_ACK_ID, ARM_SERVO_ACK_CODE, rxData, result);
        }
        return;
    }
    if ((rxHeader.IDE == CAN_ID_EXT) && (id == 0x700U) &&
        (rxHeader.DLC == 8U) && (rxData[0] == POSITION_SERVO_CMD)) {
        send_servo_ack(POSITION_SERVO_ACK_ID, POSITION_SERVO_ACK_CODE, rxData,
                       Servo_HandlePositionCanCommand(rxData));
        return;
    }

    if ((rxHeader.IDE == CAN_ID_EXT) && (id == 0x700U) &&
        (rxHeader.DLC == 8U) && (rxData[0] == 0x20U)) {
        Servo_HandleCanCommand(rxData);
        return;
    }

    /* Legacy chassis frames are accepted as either standard or extended.
     * Use the same command queue, address map and X encoder as ID 0x100. */
    if ((id == ROS_CAN_CHASSIS_ID) && (rxData[0] == MOTOR_CMD_SET_VELOCITY)) {
        if ((rxHeader.DLC != 8U) || ((rxData[1] >= MOTOR_COUNT) && (rxData[1] != 0xFFU))) {
            return;
        }
        cmd.cmd = MOTOR_CMD_SET_VELOCITY;
        cmd.motor_idx = rxData[1];
        cmd.value = parse_int32_le(rxData);
        cmd.param0 = 50U;
        cmd.param1 = 0U;
        enqueue_command(&cmd);
        return;
    }

    if (rxHeader.IDE == CAN_ID_EXT) {
        /* Removed arm Y42 IDs must not be driven by stale ROS nodes. */
        if (((id >> 8) == 5U) || ((id >> 8) == 6U)) return;
        /* Opaque driver motion/AA frames bypass body-speed and sync checks.
         * Keep reads, configuration and stop bridging; chassis motion uses
         * the unchanged 0x100 or legacy 0x201 command interfaces. */
        if ((rxData[0] == 0xF5U) || (rxData[0] == 0xF6U) ||
            (rxData[0] == 0xFAU) || (rxData[0] == 0xFBU) ||
            (rxData[0] == 0xFDU) || (rxData[0] == 0xFFU) ||
            (rxData[0] == 0xAAU) || (rxData[0] == 0x9AU)) {
            motor_record_rx_result(0U);
            return;
        }
        CAN_TxHeaderTypeDef header = {0};
        HAL_StatusTypeDef status;
        header.ExtId = rxHeader.ExtId;
        header.IDE = CAN_ID_EXT;
        header.RTR = CAN_RTR_DATA;
        header.DLC = rxHeader.DLC;
        status = can_transport_send(&hcan2, &header, rxData);
        motor_record_rx_result((status == HAL_OK) ? 1U : 0U);
        if (status == HAL_OK) { motor_set_ros_alive(); }
        return;
    }

    if ((rxHeader.IDE != CAN_ID_STD) || (id != ROS_CAN_CMD_ID) || (rxHeader.DLC != 8U)) {
        return;
    }
    cmd.cmd = rxData[0];
    cmd.motor_idx = rxData[1];
    cmd.value = parse_int32_le(rxData);
    cmd.param0 = rxData[6];
    cmd.param1 = rxData[7];
    cmd.source = MOTOR_CMD_SOURCE_REMOTE;

    if ((cmd.cmd < MOTOR_CMD_SET_VELOCITY) || (cmd.cmd > MOTOR_CMD_HEARTBEAT)) {
        return;
    }
    if ((cmd.cmd != MOTOR_CMD_SYNC_START) &&
        (cmd.cmd != MOTOR_CMD_SET_RESPONSE_POLICY) &&
        (cmd.cmd != MOTOR_CMD_SET_REPORT_MASK) &&
        (cmd.cmd != MOTOR_CMD_SET_ADDRESS_MAP) &&
        (cmd.cmd != MOTOR_CMD_HEARTBEAT) &&
        (cmd.motor_idx >= MOTOR_COUNT) && (cmd.motor_idx != 0xFFU)) {
        return;
    }
    enqueue_command(&cmd);
}
