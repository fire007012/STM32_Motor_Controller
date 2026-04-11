#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ROS_CAN_CMD_ID 0x100U
#define ROS_CAN_STATUS_ID 0x101U
#define ROS_CAN_ACK_ID 0x102U
#define ROS_CAN_STATS_ID 0x103U

/* ROS command frame format (8 bytes)
 * byte0: cmd
 *   0x01 set velocity, value=int32 rpm, param0=accel level, param1=sync(0 immediate/1 cache)
 *   0x02 set position, value=int32 pulse, param0=accel level, param1[1:0]=mode, param1[2]=sync
 *   0x03 estop, value ignored
 *   0x04 set profile, value=uint16 speed rpm, param0=accel level
 *   0x05 trigger sync start, value ignored
 *   0x06 set response policy, param0=1 wait-ack / 0 fire-and-forget
 *   0x07 set report mask, value[7:0]=bitmask(1 basic,2 pos,4 vel,8 target)
 *   0x08 set address map, value byte0..3 = motor0..3 slave address
 * byte1: motor index 0..3 or 0xFF for all motors
 * byte2..5: value (int32 little-endian)
 * byte6: param0
 * byte7: param1
 *
 * Status frame note:
 *   type 0x03 current velocity and type 0x04 target velocity use 0.1 RPM units.
 */

void CAN1_RxCallback(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[8]);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
