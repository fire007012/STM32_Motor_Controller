#ifndef CHASSIS_SAFETY_CONFIG_H
#define CHASSIS_SAFETY_CONFIG_H

/* STM32 keeps link/fault/driver safeguards. ROS owns navigation/braking,
 * per-wheel command freshness, feedback freshness and all chassis geometry. */
#define CHASSIS_RANGE_MIN_MM           50U /* sensor validity, not parking */
#define CHASSIS_RANGE_MAX_MM           3000U
#define CHASSIS_COMMAND_TIMEOUT_MS     500U /* queued/in-flight request budget */
#define CHASSIS_HEARTBEAT_TIMEOUT_MS   500U
#define CHASSIS_CONTROL_PERIOD_MS      10U
#define DISTANCE_I2C_TIMEOUT_MS        10U
#define DISTANCE_BOOT_TIMEOUT_MS       100U
#define DISTANCE_SAMPLE_TIMEOUT_MS     150U
#define DISTANCE_INTERMEASUREMENT_MS   50U
#define DISTANCE_POLL_PERIOD_MS        5U
#define DISTANCE_TIMING_BUDGET_US      33000U
#define DISTANCE_RECOVERY_PERIOD_MS    1000U
#define DISTANCE_ERROR_REPORT_MS       1000U

#endif
