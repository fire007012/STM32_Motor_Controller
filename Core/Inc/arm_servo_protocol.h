#ifndef ARM_SERVO_PROTOCOL_H
#define ARM_SERVO_PROTOCOL_H
#include <stdint.h>
#define ARM_SERVO_CAN_ID        0x701U
#define ARM_SERVO_ACK_ID        0x702U
#define ARM_SERVO_CMD_SET       0x21U
#define ARM_SERVO_CMD_HOLD      0x23U
#define ARM_SERVO_CMD_RELEASE   0x24U
#define ARM_SERVO_ACK_CODE      0xA1U
#define POSITION_SERVO_ACK_ID   0x703U
#define POSITION_SERVO_CMD      0x22U
#define POSITION_SERVO_ACK_CODE 0xA2U
/* SET: [0x21, mask=1/2/3, angle1_i16_le, angle2_i16_le, seq, sum8].
 * Angles are signed tenths of a degree relative to each calibrated zero.
 * HOLD/RELEASE: [opcode,0,0,0,0,0,seq,sum8]. HOLD retains PWM/blocks keys;
 * RELEASE relinquishes local key ownership without moving.
 * ACK: [ack_code,seq,result,opcode,mask,0,0,sum8], result 0=accepted,
 * 1=bad packet/command, 2=range, 3=not ready. NOT physical arrival feedback.
 * SG90 0x700/0x22: [0x22,mask,PB0_deg,PD14_deg,0,0,seq,sum8]. */
#endif
