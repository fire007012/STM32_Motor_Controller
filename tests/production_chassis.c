/* Run the real production matrix against the ROS-supplied motor mapping.
 * Reuse protocol mocks, not the synthetic test calibration. */
#define main unused_synthetic_regression_main
#include "can_regression.c"
#undef main

static void expect_body(const float rpm[4], float vx, float vy, float wz)
{
    chassis_velocity_t body;
    chassis_velocity_from_rpm(rpm, &body);
    assert(fabsf(body.vx-vx)<0.000001f);
    assert(fabsf(body.vy-vy)<0.000001f);
    assert(fabsf(body.wz-wz)<0.000001f);
}

static void test_ros_mapping(void)
{
    assert(CHASSIS_KINEMATICS_READY == 1U);
    assert(CHASSIS_KINEMATICS_L_M == 0.20f);
    const float c = CHASSIS_MOTOR_RPM_TO_WHEEL_MPS;
    float p = 0.20f/c, turn = -CHASSIS_KINEMATICS_L_M*0.20f/c;
    expect_body((float[]){0,p,0,-p}, .20f,0,0);
    expect_body((float[]){p,0,-p,0}, 0,.20f,0);
    expect_body((float[]){-p,0,p,0}, 0,-.20f,0);
    expect_body((float[]){0,-p,0,p}, -.20f,0,0);
    expect_body((float[]){turn,turn,turn,turn}, 0,0,.20f);
    /* STM32 motor basis: +motor0=+y, +motor1=+x, +motor2=-y, +motor3=-x. */
    const float h=c/2.0f, k=-c/(4.0f*.20f);
    expect_body((float[]){1,0,0,0},0,h,k);
    expect_body((float[]){0,1,0,0},h,0,k);
    expect_body((float[]){0,0,1,0},0,-h,k);
    expect_body((float[]){0,0,0,1},-h,0,k);
    expect_body((float[]){0,25,0,-25},.19634954f,0,0);
    expect_body((float[]){-5,-5,-5,-5},0,0,.19634954f);
    /* Combined ROS forward equation, reordered exactly ONCE as [1,0,3,2]. */
    float vx=.20f, vy=-.10f, wz=.30f, l=CHASSIS_KINEMATICS_L_M;
    expect_body((float[]){(vy-l*wz)/c,(vx-l*wz)/c,
                          (-vy-l*wz)/c,(-vx-l*wz)/c},vx,vy,wz);
    puts("PASS production K: r=.075, L=.20, direct drive, STM32 [1,0,3,2] mapping and signed ROS roundtrips");
}

static void test_production_directional_stops(void)
{
    const uint8_t ids[4]={0,1,2,5};
    const int direction[4][4]={{0,1,0,-1},{1,0,-1,0},{-1,0,1,0},{0,-1,0,1}};
    for (unsigned speed=25; speed<=32; speed+=7) {
        float v=CHASSIS_MOTOR_RPM_TO_WHEEL_MPS*(float)speed;
        unsigned stop=(unsigned)chassis_collision_stop_mm(v);
        assert(stop == (speed==25 ? 158U : 181U));
        for (unsigned d=0;d<4;++d) {
            reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
            unsigned raw=stop+(d?50U:0U);
            range_sample(ids[d],(uint16_t)(raw+1U),1U);
            assert(group(direction[d][0]*(int)speed,direction[d][1]*(int)speed,
                         direction[d][2]*(int)speed,direction[d][3]*(int)speed)==HAL_OK);
            drain();
            range_sample(ids[d],(uint16_t)raw,1U);
            motor_safety_poll(); expect_actual_stop();
        }
    }
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    for(unsigned i=0;i<4;++i) range_sample(ids[i],0xFFFFU,4U);
    assert(group(0,25,0,-25)==HAL_OK); drain();
    motor_safety_poll(); assert(motors[1].target_velocity==250);
    range_sample(0,50,1U); motor_safety_poll(); expect_actual_stop();
    /* Full atomic lateral request has zero yaw; close front does not block. */
    assert(group(25,0,-25,0)==HAL_OK); drain();
    range_sample(3,50,1U); range_sample(4,50,1U);
    motor_safety_poll(); assert(motors[0].target_velocity==250);
    puts("PASS production four-direction braking boundaries, missing ranges, zero-yaw lateral departure and arm exclusion");
}

/* Exercise the unchanged 0x100 fields, queue, application and X driver bytes. */
static HAL_StatusTypeDef send_ros_velocity(uint8_t motor, int32_t rpm, uint8_t sync)
{
    CAN_RxHeaderTypeDef header={.StdId=ROS_CAN_CMD_ID,.IDE=CAN_ID_STD,.RTR=CAN_RTR_DATA,.DLC=8};
    uint32_t raw=(uint32_t)rpm;
    uint8_t data[8]={1U,motor,(uint8_t)raw,(uint8_t)(raw>>8),
                    (uint8_t)(raw>>16),(uint8_t)(raw>>24),50U,sync};
    Motor_Command_t cmd; HAL_StatusTypeDef result;
    CAN1_RxCallback(header,data);
    assert(motor_process_next_command(&cmd,&result));
    assert(cmd.motor_idx==motor && cmd.value==rpm && cmd.param1==sync);
    return result;
}

static void immediate_ros_group(const int32_t rpm[4])
{
    /* ROS logical wheels 0..3 map to STM32 motor indices [1,0,3,2]. */
    static const uint8_t order[4]={1,0,3,2};
    for(unsigned i=0;i<4;++i) {
        uint8_t motor=order[i];
        assert(send_ros_velocity(motor,rpm[motor],0U)==HAL_OK);
        unsigned before=sent_count[1];
        drain();
        assert(sent_count[1]==before+1U); /* immediate means transmitted now */
        CAN_TxFrame_t *frame=&sent[1][before];
        assert(frame->header.ExtId==(uint32_t)(motor+1U)<<8);
        assert(frame->data[0]==0xF6U && frame->data[6]==0U);
        assert(frame->data[1]==(rpm[motor]<0));
    }
}

static void test_ros_start_and_immediate_updates(void)
{
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0,250,1U);
    assert(group(0,6,0,-6)==HAL_OK); drain(); /* .04712 m/s calibration */
    for(unsigned mm=270;mm>=250;--mm) {
        range_sample(0,(uint16_t)mm,1U);
        immediate_ros_group((int32_t[]){0,6,0,-6});
    }
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0,250,1U);
    for(uint8_t i=0;i<4;++i) assert(send_ros_velocity(i,0,1)==HAL_OK);
    assert(motor_request_local_sync_start(123U));
    Motor_Command_t cmd; HAL_StatusTypeDef status;
    assert(motor_process_next_command(&cmd,&status) && status==HAL_OK);
    drain();
    expect_frame(1,sent_count[1]-1U,0,(uint8_t[]){0xFF,0x66,0x6B},3);
    immediate_ros_group((int32_t[]){0,25,0,-25});
    immediate_ros_group((int32_t[]){25,0,-25,0});
    immediate_ros_group((int32_t[]){0,-25,0,25});
    immediate_ros_group((int32_t[]){-5,-5,-5,-5});
    motor_safety_poll();
    assert(motors[0].target_velocity==-50);
    puts("PASS ROS zero-cache/button startup, running immediate per-wheel updates, 250 mm calibration and turning");
}

static void test_immediate_transient_is_real_motion(void)
{
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    range_sample(0,90,1U);
    /* First wheel of an immediate lateral update alone requests yaw.
     * Do not suppress the <=100 mm turning guard by pretending later wheels
     * already arrived. ROS can send an atomic sync group to avoid this. */
    assert(send_ros_velocity(0,25,0)==HAL_ERROR); expect_actual_stop();
    assert(group(25,0,-25,0)==HAL_OK); drain();
    /* Actual rotation at the same distance still stops. */
    assert(group(-5,-5,-5,-5)==HAL_ERROR); expect_actual_stop();
    puts("PASS real immediate yaw transients remain protected; atomic lateral group escapes a 90 mm front obstacle");
}

static void test_production_measured_speed(void)
{
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    uint8_t got=0U;
    const int32_t wire[4]={0,320,0,-320}; /* +.251327 m/s, wire = 0.1 RPM */
    for (unsigned step=0;step<40 && got!=15U;++step) {
        motor_update_status(1); drain();
        CAN_TxFrame_t *frame=&sent[1][sent_count[1]-1U];
        uint8_t address=(uint8_t)(frame->header.ExtId>>8);
        if (frame->data[0]==0x35U) {
            int32_t value=wire[address-1U];
            uint16_t magnitude=(uint16_t)(value<0 ? -value : value);
            reply(address,(uint8_t[]){0x35,(uint8_t)(value<0),
                                      (uint8_t)(magnitude>>8),(uint8_t)magnitude,0x6B},5);
            got|=(uint8_t)(1U<<(address-1U));
        } else if (frame->data[0]==0x36U) reply(address,(uint8_t[]){0x36,0,0,0,0,0,0x6B},7);
        else reply(address,(uint8_t[]){0x3A,0x83,0x6B},3);
        tick+=20U;
    }
    assert(got==15U);
    range_sample(0,170,1U);
    assert(group(0,6,0,-6)==HAL_ERROR); expect_actual_stop();
    /* Request .04712 m/s alone would allow 170 mm; measured .251327 must stop. */
    tick+=CHASSIS_FEEDBACK_FRESH_MS+1U;
    range_sample(0,0xFFFFU,4U);
    assert(group(0,6,0,-6)==HAL_OK); drain();
    range_sample(0,170,1U); motor_safety_poll();
    assert(motors[1].target_velocity==60 && motors[3].target_velocity==-60);
    puts("PASS production signed 0.1 RPM feedback uses the same K, braking speed maximum and stale-data exclusion");
}

int main(void)
{
    test_ros_mapping();
    test_production_directional_stops();
    test_ros_start_and_immediate_updates();
    test_immediate_transient_is_real_motion();
    test_production_measured_speed();
    puts("Production ROS kinematics regression tests passed (software only).");
    return 0;
}
