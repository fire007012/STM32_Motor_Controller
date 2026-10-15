#ifndef CHASSIS_SAFETY_CONFIG_H
#define CHASSIS_SAFETY_CONFIG_H

/* Distances are mm, body speeds m/s and rad/s. No ROS warning multiplier. */
#define CHASSIS_BRAKE_DECEL_MPS2       1.0f
#define CHASSIS_REACTION_TIME_S        0.2f
#define CHASSIS_EDGE_MARGIN_MM         100.0f
#define CHASSIS_MIN_STOP_MM            150.0f
#define CHASSIS_ROTATION_STOP_MM       100.0f
#define CHASSIS_RELEASE_MM             30.0f
#define CHASSIS_SENSOR_INSETS_MM       {0U, 50U, 50U, 50U} /* front,left,right,rear */
#define CHASSIS_RANGE_MIN_MM           50U
#define CHASSIS_RANGE_MAX_MM           3000U
#define CHASSIS_RANGE_FRESH_MS         200U
#define CHASSIS_FEEDBACK_FRESH_MS      200U
#define CHASSIS_COMMAND_TIMEOUT_MS     500U
#define CHASSIS_HEARTBEAT_TIMEOUT_MS   500U
#define CHASSIS_CONTROL_PERIOD_MS      10U
#define CHASSIS_SPEED_EPSILON          0.00001f
#define CHASSIS_WHEEL_DIAMETER_M       0.150f /* confirmed by user */
#define CHASSIS_MOTOR_TO_WHEEL_RATIO   1.0f   /* direct drive, no reducer */
#define CHASSIS_MOTOR_RPM_TO_WHEEL_MPS \
    (3.14159265358979323846f * CHASSIS_WHEEL_DIAMETER_M / \
     (60.0f * CHASSIS_MOTOR_TO_WHEEL_RATIO))
#define CHASSIS_KINEMATICS_L_M         0.200f /* latest user-confirmed L; update ROS from .18 */
#define DISTANCE_I2C_TIMEOUT_MS        10U
#define DISTANCE_BOOT_TIMEOUT_MS       100U
#define DISTANCE_SAMPLE_TIMEOUT_MS     150U
#define DISTANCE_INTERMEASUREMENT_MS   50U
#define DISTANCE_POLL_PERIOD_MS        5U
#define DISTANCE_TIMING_BUDGET_US      33000U
#define DISTANCE_RECOVERY_PERIOD_MS    1000U
#define DISTANCE_ERROR_REPORT_MS       1000U

/* Matches the ROS SOURCE configuration supplied by the user, not a physical
 * calibration: r=.075, L=.20 (user correction to ROS's previous .18);
 * can_interface.launch logical
 * wheel -> STM32 motor = [1,0,3,2], signs=[1,1,1,1]. Motor addresses stay 1..4.
 * Omni wheel type confirmed by the user; axial four-omni ROS equations:
 *   vx=c/2*(motor1-motor3), vy=c/2*(motor0-motor2),
 *   wz=-c/(4L)*(motor0+motor1+motor2+motor3), c=pi*diameter/(60*ratio).
 * READY means a code-compatible matrix is available; actual wheel positions,
 * roller installation and ground directions still need calibration. Body radius .25 m
 * MUST NOT replace L. A board-specific header
 * can override READY and the matrix, including explicit READY=0 inhibition.
 */
#ifdef CHASSIS_KINEMATICS_HEADER
#include CHASSIS_KINEMATICS_HEADER
#else
#define CHASSIS_KINEMATICS_READY 1U
#define CHASSIS_RPM_TO_BODY_MATRIX \
    {{0.0f, CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/2.0f, 0.0f, -CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/2.0f}, \
     {CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/2.0f, 0.0f, -CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/2.0f, 0.0f}, \
     {-CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/(4.0f*CHASSIS_KINEMATICS_L_M), \
      -CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/(4.0f*CHASSIS_KINEMATICS_L_M), \
      -CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/(4.0f*CHASSIS_KINEMATICS_L_M), \
      -CHASSIS_MOTOR_RPM_TO_WHEEL_MPS/(4.0f*CHASSIS_KINEMATICS_L_M)}}
#endif
#endif
