/* Explicitly unavailable calibration: exercise the fail-closed override. */
#define CHASSIS_KINEMATICS_READY 0U
#define CHASSIS_RPM_TO_BODY_MATRIX \
    {{0.0f,0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f,0.0f}, {0.0f,0.0f,0.0f,0.0f}}
