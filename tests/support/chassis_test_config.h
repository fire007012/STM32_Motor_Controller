/* SYNTHETIC HOST TEST MODEL ONLY. Not the real chassis calibration.
 * vx=.001 * mean(RPM), orthogonal signed rows for vy and wz. */
#define CHASSIS_KINEMATICS_READY 1U
#define CHASSIS_RPM_TO_BODY_MATRIX \
    {{0.00025f,0.00025f,0.00025f,0.00025f}, \
     {-0.00025f,0.00025f,0.00025f,-0.00025f}, \
     {-0.0025f,0.0025f,-0.0025f,0.0025f}}
