#include "can_protocol.h"

#include "motor_control.h"

static int32_t parse_int32_le(const uint8_t data[8])
{
    uint32_t v;

    v = ((uint32_t)data[2]) |
        ((uint32_t)data[3] << 8) |
        ((uint32_t)data[4] << 16) |
        ((uint32_t)data[5] << 24);
    return (int32_t)v;
}

void CAN1_RxCallback(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8])
{
    Motor_Command_t cmd;
    uint8_t enqueue_ok;

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
