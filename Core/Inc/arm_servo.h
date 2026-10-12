#ifndef ARM_SERVO_H
#define ARM_SERVO_H

#include "main.h"
#include "arm_servo_protocol.h"
#include "arm_servo_config.h"

/* TIM8: PC6=CH1, PC7=CH2, PC8=CH3 (reserved), PC9=CH4 (reserved).
 * Init returns two active channels to logical zero. No position feedback. */
HAL_StatusTypeDef ArmServo_Init(void);
/* Called only for extended 0x701, DLC=8. Returns ACK result. */
uint8_t ArmServo_HandleCanCommand(const uint8_t data[8]);
void ArmServo_PollButtons(uint8_t k0_pressed, uint8_t k1_pressed);
/* One-based channel 1/2 only; returns commanded logical angle, not measured. */
int16_t ArmServo_GetTargetAngle(uint8_t channel);

#endif
