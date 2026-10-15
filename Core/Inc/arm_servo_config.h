#ifndef ARM_SERVO_CONFIG_H
#define ARM_SERVO_CONFIG_H

/* K0/K1 selection: 1=DS3230MG, 0=SG90. Both PWM and CAN drivers stay
 * initialized in either mode. Remote arm ownership blocks local arm keys. */
#define ARM_SERVO_KEY_TEST_ENABLE       1U //测试sg90 0为sg90
#define ARM_SERVO_CHANNEL_COUNT         4U
#define ARM_SERVO_ACTIVE_COUNT          3U
#define ARM_SERVO_STEP_DEG              5  //每次转多少度
#define ARM_SERVO_FRAME_US              20000U

/* WK_UP dispenser sequence; serviced non-blocking from the RTOS task. */
#define ARM_SERVO_WK_DEMO_ENABLE        1U
#define ARM_SERVO_WK_PLACE_DEG          30
#define ARM_SERVO_WK_HOME_DEG           0
#define ARM_SERVO_WK_MOVE_MS            3000U
#define ARM_SERVO_WK_HOLD_MS            1000U
#define ARM_SERVO_WK_SERVO_ANGLE         0
#define ARM_SERVO_WK_SERVO_RETURN_ANGLE  180
#define ARM_SERVO_WK_SERVO_HOLD_MS       3000U
#define ARM_SERVO_WK_SERVO_RETURN_MS     1000U

/* Confirm your DS3230MG's 180/270-degree variant before powered testing.
 * Logical zero means the calibrated pulse below, not the shaft's endpoint.
 * Directions are viewed from the shaft; +1 means increasing pulse width.
 * Change a direction to -1 for an independently mirrored installation.
 * The +/-80 limits are software bounds, NOT verified arm clearances. */
/* CH1 / PC6: RIGHT arm, replaces Y42 ID5. Desired forward is CCW.
 * CH2 / PC7: LEFT arm, replaces Y42 ID6. Desired forward is CW.
 * Positive logical angles mean forward on BOTH arms. Directions below are
 * pulse signs, not a universal CW/CCW guarantee: verify each at +5 degrees.
 * Mapping restored before commit a5c3e4a: PC6 increases pulse, PC7 decreases
 * pulse. K1 and positive angles/WK_UP are forward; K0 is backward. */
#define ARM_SERVO_1_TRAVEL_DEG           180U
#define ARM_SERVO_1_MIN_PULSE_US         500U
#define ARM_SERVO_1_MAX_PULSE_US         2500U
#define ARM_SERVO_1_ZERO_PULSE_US        1500U
#define ARM_SERVO_1_DIRECTION            1
#define ARM_SERVO_1_MIN_ANGLE_DEG       (-80)
#define ARM_SERVO_1_MAX_ANGLE_DEG        80

#define ARM_SERVO_2_TRAVEL_DEG           180U
#define ARM_SERVO_2_MIN_PULSE_US         500U
#define ARM_SERVO_2_MAX_PULSE_US         2500U
#define ARM_SERVO_2_ZERO_PULSE_US        1500U
#define ARM_SERVO_2_DIRECTION           (-1)
#define ARM_SERVO_2_MIN_ANGLE_DEG       (-80)
#define ARM_SERVO_2_MAX_ANGLE_DEG        80

/* CH3 / PC8 is the second RIGHT-side servo and always follows CH1 / PC6.
 * Aliases keep pulse endpoints, zero, direction and limits identical even
 * when the PC6 calibration changes. CH3 is not an independent CAN axis. */
#define ARM_SERVO_3_TRAVEL_DEG          ARM_SERVO_1_TRAVEL_DEG
#define ARM_SERVO_3_MIN_PULSE_US        ARM_SERVO_1_MIN_PULSE_US
#define ARM_SERVO_3_MAX_PULSE_US        ARM_SERVO_1_MAX_PULSE_US
#define ARM_SERVO_3_ZERO_PULSE_US       ARM_SERVO_1_ZERO_PULSE_US
#define ARM_SERVO_3_DIRECTION          ARM_SERVO_1_DIRECTION
#define ARM_SERVO_3_MIN_ANGLE_DEG       ARM_SERVO_1_MIN_ANGLE_DEG
#define ARM_SERVO_3_MAX_ANGLE_DEG       ARM_SERVO_1_MAX_ANGLE_DEG

/* PC9/CH4 remains configured but never started. Four-servo operation
 * requires implementing its side/axis mapping, not just changing this count. */
#if ARM_SERVO_CHANNEL_COUNT != 4U || ARM_SERVO_ACTIVE_COUNT != 3U
#error "This stage implements three active servos and four configured PWM channels"
#endif
#if ARM_SERVO_KEY_TEST_ENABLE > 1U
#error "ARM_SERVO_KEY_TEST_ENABLE must be 0 or 1"
#endif

#endif
