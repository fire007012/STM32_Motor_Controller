/* Reuse the HAL/queue fixture with an explicitly unavailable matrix override. */
#define main unused_configured_regression_main
#include "can_regression.c"
#undef main

int main(void)
{
    assert(CHASSIS_KINEMATICS_READY == 0U);
    reset();
    zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    assert(group(200,200,200,200) == HAL_ERROR);
    expect_actual_stop();
    /* Unconfigured geometry is an explicit motion configuration error.
     * It never becomes a fake distance or a ROS emergency event. */
    assert(group(0,0,0,0) == HAL_OK);
    assert(motor_is_ros_ready(CHASSIS_HEARTBEAT_TIMEOUT_MS));
    puts("PASS unavailable matrix override rejects motion and preserves heartbeat/zero commands without fake emergency");
    return 0;
}
