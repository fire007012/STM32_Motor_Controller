/* Targeted checks for terminal command correlation and button cache lifetime. */
static void test_async_motion_outcomes(void)
{
    Motor_Command_t in={.cmd=MOTOR_CMD_SET_VELOCITY,.motor_idx=2,.value=-5,
        .param0=50,.param1=1,.seq=0x1F79}, out;
    HAL_StatusTypeDef status;
    Motor_CommStats_t before, after;
    reset();
    motor_get_comm_stats(&before);
    assert(motor_enqueue_command_from_isr(&in));
    assert(motor_process_next_command(&out,&status) && status==HAL_BUSY);
    assert(!motor_take_command_result(&out,&status));
    /* Neither an old reply before TX, another wheel nor old completion
     * can confirm the current address-3 F6 command. */
    reply(3,(uint8_t[]){0xF6,2,0x6B},3);
    assert(!motor_take_command_result(&out,&status));
    drain();
    reply(2,(uint8_t[]){0xF6,2,0x6B},3);
    reply(3,(uint8_t[]){0xFB,2,0x6B},3);
    reply(3,(uint8_t[]){0xF6,0x9F,0x6B},3);
    assert(!motor_take_command_result(&out,&status));
    reply(3,(uint8_t[]){0xF6,2,0x6B},3);
    assert(motor_take_command_result(&out,&status) && status==HAL_OK && out.seq==0x1F79);
    assert(!motor_take_command_result(&out,&status));
    motor_get_comm_stats(&after);
    assert(after.exec_ok_count==before.exec_ok_count+1 && after.exec_fail_count==before.exec_fail_count);

    /* Real native rejection is terminal, correlated, counted once, stopped. */
    in.seq++;
    assert(motor_enqueue_command_from_isr(&in));
    assert(motor_process_next_command(&out,&status)); drain();
    reply(3,(uint8_t[]){0xF6,0xE2,0x6B},3);
    assert(motor_take_command_result(&out,&status) && status==HAL_ERROR && out.seq==in.seq);
    assert(motor_last_failure.seq==in.seq && motor_last_failure.motor_idx==2 &&
           motor_last_failure.reason==MOTOR_REASON_NATIVE_REJECT &&
           motor_last_failure.native_result==0xE2 && motor_last_failure.slave==3);
    expect_actual_stop();

    /* New command waits while old late ACKs drain; no replay of failed F6. */
    unsigned before_new=sent_count[1];
    in.seq++;
    assert(motor_enqueue_command_from_isr(&in));
    assert(motor_process_next_command(&out,&status));
    reply(3,(uint8_t[]){0xF6,2,0x6B},3);
    drain(); assert(sent_count[1]==before_new);
    tick+=50; zdt_can_driver_timeout_poll(); drain();
    assert(sent_count[1]==before_new+1);
    reply(3,(uint8_t[]){0xF6,2,0x6B},3);
    assert(motor_take_command_result(&out,&status) && status==HAL_OK && out.seq==in.seq);

    /* Lost ACK stops; it cannot release the next wheel's queued motion. */
    reset();
    in.param1=0;
    for(uint8_t i=0;i<4;++i) { in.motor_idx=i; in.seq=(uint16_t)(100+i); assert(motor_enqueue_command_from_isr(&in)); }
    while(motor_process_next_command(&out,&status)) {}
    drain(); assert(sent_count[1]==1);
    tick=50; zdt_can_driver_timeout_poll(); drain(); assert(sent_count[1]==1);
    unsigned failures=0;
    while(motor_take_command_result(&out,&status)) { assert(status!=HAL_OK); failures++; }
    assert(failures==1 && command_count==0);
    expect_actual_stop();
    unsigned after_stop=sent_count[1];
    tick+=100; zdt_can_driver_timeout_poll(); drain();
    assert(sent_count[1]==after_stop);

    /* Temporary HAL_BUSY is pending; real HAL_ERROR is terminal, with seq. */
    reset(); in.motor_idx=2; in.param1=1; in.seq=150;
    assert(motor_enqueue_command_from_isr(&in));
    assert(motor_process_next_command(&out,&status));
    busy_next[1]=1; free_mailboxes[1]=3; can_transport_service();
    assert(!motor_take_command_result(&out,&status));
    fail_next[1]=1; can_transport_service();
    assert(motor_take_command_result(&out,&status) && status==HAL_ERROR && out.seq==150);
    assert(motor_last_failure.seq==150 && motor_last_failure.reason==MOTOR_REASON_TRANSMIT);
    expect_actual_stop();

    /* Urgent stop cancels a pending transaction before F6 reaches the bus. */
    reset(); in.motor_idx=0; in.param1=0;
    assert(motor_enqueue_command_from_isr(&in));
    assert(motor_process_next_command(&out,&status));
    Motor_Command_t stop={.cmd=MOTOR_CMD_ESTOP,.seq=200};
    assert(motor_enqueue_command_from_isr(&stop));
    assert(!motor_process_next_command(&out,&status));
    assert(motor_take_command_result(&out,&status) && status==HAL_ERROR);
    assert(pump_one_command(&out,&status) && out.cmd==MOTOR_CMD_ESTOP && status==HAL_OK);
    for(unsigned i=0;i<sent_count[1];++i) assert(sent[1][i].data[0]==0xFE);
    puts("PASS async terminal results, seq/index/native rejection diagnostics, stale/other reply filtering, quiet drain, no motion retry and priority ESTOP");
}

static void test_button_cache_wait_and_failure(void)
{
    Motor_Command_t in={.cmd=MOTOR_CMD_SET_VELOCITY,.value=0,.param0=50,.param1=1},out;
    HAL_StatusTypeDef status;
    reset();
    /* Default WAIT_ACK: four distinct native confirmations before button. */
    for(uint8_t i=0;i<4;++i) {
        in.motor_idx=i; in.seq=(uint16_t)(300+i);
        assert(motor_enqueue_command_from_isr(&in));
        assert(pump_one_command(&out,&status) && status==HAL_OK);
    }
    for(unsigned i=0;i<20;++i) { tick+=100; motor_set_ros_alive(); motor_safety_poll(); }
    assert(motor_request_local_sync_start(0x1504));
    assert(motor_process_next_command(&out,&status) && status==HAL_BUSY);
    assert(!motor_take_command_result(&out,&status));
    drain(); zdt_can_driver_timeout_poll();
    assert(!motor_take_command_result(&out,&status));
    reply(2,(uint8_t[]){0xFF,2,0x6B},3);
    assert(!motor_take_command_result(&out,&status));
    reply(1,(uint8_t[]){0xFF,2,0x6B},3);
    assert(motor_take_command_result(&out,&status) && status==HAL_OK && out.seq==0x1504);
    expect_frame(1,sent_count[1]-1U,0,(uint8_t[]){0xFF,0x66,0x6B},3);

    /* Losing heartbeat while armed invalidates all cached wheels. */
    reset(); zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
    for(uint8_t i=0;i<4;++i) assert(motor_set_velocity_ex(i,0,50,1)==HAL_OK);
    tick=501; motor_safety_poll(); drain(); motor_set_ros_alive();
    unsigned stopped=sent_count[1];
    assert(motor_request_local_sync_start(0x1505));
    assert(motor_process_next_command(&out,&status));
    assert(motor_take_command_result(&out,&status) && status==HAL_ERROR);
    assert(motor_last_failure.seq==0x1505 && motor_last_failure.reason==MOTOR_REASON_CACHE_INCOMPLETE);
    expect_actual_stop();
    assert(sent_count[1]==stopped); /* incomplete cache alone does not stop again */
    puts("PASS confirmed cache survives human wait; heartbeat loss invalidates cache; incomplete start reports only");
}
