#ifndef ARM_SERVO_CONFIG_H
#define ARM_SERVO_CONFIG_H

/* K0/K1 selection: 1=DS3230MG, 0=SG90. Both PWM and CAN drivers stay
 * initialized in either mode. Remote arm ownership blocks local arm keys. */
#define ARM_SERVO_KEY_TEST_ENABLE       1U //测试sg90 0为sg90
#define ARM_SERVO_CHANNEL_COUNT         4U
#define ARM_SERVO_ACTIVE_COUNT          2U
#define ARM_SERVO_STEP_DEG              5  //每次转多少度
#define ARM_SERVO_FRAME_US              20000U

/* Confirm your DS3230MG's 180/270-degree variant before powered testing.
 * Logical zero means the calibrated pulse below, not the shaft's endpoint.
 * Directions are viewed from the shaft; +1 means increasing pulse width.
 * Change a direction to -1 for an independently mirrored installation.
 * The initial +/-60 limits are bench limits, NOT verified arm clearances. */
#define ARM_SERVO_1_TRAVEL_DEG           180U
#define ARM_SERVO_1_MIN_PULSE_US         500U
#define ARM_SERVO_1_MAX_PULSE_US         2500U
#define ARM_SERVO_1_ZERO_PULSE_US        1500U
#define ARM_SERVO_1_DIRECTION           1
#define ARM_SERVO_1_MIN_ANGLE_DEG       (-60)
#define ARM_SERVO_1_MAX_ANGLE_DEG        60

#define ARM_SERVO_2_TRAVEL_DEG           180U
#define ARM_SERVO_2_MIN_PULSE_US         500U
#define ARM_SERVO_2_MAX_PULSE_US         2500U
#define ARM_SERVO_2_ZERO_PULSE_US        1500U
#define ARM_SERVO_2_DIRECTION           1
#define ARM_SERVO_2_MIN_ANGLE_DEG       (-60)
#define ARM_SERVO_2_MAX_ANGLE_DEG        60

/* PC8/CH3 and PC9/CH4 are configured but NOT started or position-controlled.
 * Do not enable four-servo operation by changing ACTIVE_COUNT alone. */
#if ARM_SERVO_CHANNEL_COUNT != 4U || ARM_SERVO_ACTIVE_COUNT != 2U
#error "This stage implements exactly two active servos and four configured PWM channels"
#endif
#if ARM_SERVO_KEY_TEST_ENABLE > 1U
#error "ARM_SERVO_KEY_TEST_ENABLE must be 0 or 1"
#endif

#endif
