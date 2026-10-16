static void test_native_forward_backoff_and_priority(void)
{
    CAN_TxHeaderTypeDef telemetry = {.StdId = 0x101U, .DLC = 8U};
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    CAN_BusDiagnostics_t diag;
    reset();
    assert(can_transport_send_latest(&hcan1, &telemetry, (uint8_t[8]){1U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_TERR0;
    can_transport_service();
    /* Actual CAN2 receive/bridge path, including both multipart IDs. */
    inject(&hcan2, 0x300U, (uint8_t[]){0xF6U, 2U, 0x6BU}, 3U);
    inject(&hcan2, 0x301U, (uint8_t[]){0x36U, 0U, 1U, 0x6BU}, 4U);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(can_transport_send_reply(&hcan1, &ack, (uint8_t[8]){9U}) == HAL_OK);
    assert(can_transport_send_latest(&hcan1, &telemetry, (uint8_t[8]){2U}, 1U) == HAL_OK);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.forward_pending == 2U && diag.backoff_rejected == 0U);
    assert(diag.tx_failures == 1U && diag.tx_error == 1U);
    tick = CAN_ROS_TX_BACKOFF_MS - 1U;
    can_transport_service();
    assert(sent_count[0] == 1U);
    tick++;
    drain();
    assert(sent_count[0] == 5U && sent[0][1].header.StdId == 0x102U);
    expect_frame(0U, 2U, 0x300U, (uint8_t[]){0xF6U, 2U, 0x6BU}, 3U);
    expect_frame(0U, 3U, 0x301U, (uint8_t[]){0x36U, 0U, 1U, 0x6BU}, 4U);
    assert(sent[0][4].header.StdId == 0x101U && sent[0][4].data[0] == 2U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.forward_pending == 0U && diag.tx_failures == 1U && diag.tx_ok == 4U);
    puts("PASS native bridge retains multipart FIFO during backoff; ACK precedes native feedback and telemetry");
}

static void test_native_forward_retry_expiry_and_cancel(void)
{
    CAN_TxHeaderTypeDef native = {.ExtId = 0x200U, .IDE = CAN_ID_EXT, .DLC = 3U};
    CAN_BusDiagnostics_t diag;
    reset();
    assert(can_transport_forward(&hcan2, &native, (uint8_t[8]){0}) == HAL_ERROR);
    assert(can_transport_forward(NULL, &native, (uint8_t[8]){0}) == HAL_ERROR);
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){0xF6U, 2U, 0x6BU}) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_TERR0;
    can_transport_service();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.forward_pending == 1U && diag.forward_retries == 1U && diag.tx_failures == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 2U);
    expect_frame(0U, 1U, 0x200U, (uint8_t[]){0xF6U, 2U, 0x6BU}, 3U);

    reset();
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){1U}) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    tick = CAN_ROS_TX_TIMEOUT_MS;
    can_transport_service();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(aborted[0] == 1U && diag.forward_pending == 1U && diag.forward_retries == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 2U);

    /* Entering hardware late must not extend the reception lifetime. */
    reset();
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){1U}) == HAL_OK);
    tick = CAN_FORWARD_LIFETIME_MS - 1U;
    free_mailboxes[0] = 3U;
    can_transport_service();
    tick++;
    can_transport_service();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(aborted[0] == 1U && diag.forward_pending == 0U && diag.forward_expired == 1U);
    assert(diag.tx_failures == 1U && diag.tx_timeouts == 0U);

    /* Continuous errors must not refresh the original reception deadline. */
    reset();
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){1U}) == HAL_OK);
    for (tick = 0U; tick < 1000U; tick++) {
        free_mailboxes[0] = 3U;
        can_registers[0].TSR = CAN_TSR_TERR0;
        can_transport_service();
    }
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(sent_count[0] == 2U && diag.forward_pending == 0U);
    assert(diag.forward_expired == 1U && diag.forward_retries == 2U && diag.tx_failures == 3U);

    reset();
    tick = UINT32_MAX - 49U;
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){1U}) == HAL_OK);
    tick += CAN_FORWARD_LIFETIME_MS;
    drain();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(sent_count[0] == 0U && diag.forward_expired == 1U && diag.tx_failures == 1U);
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){2U}) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    can_transport_cancel(&hcan1);
    drain();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(sent_count[0] == 1U && diag.forward_pending == 0U && diag.canceled_frames == 1U);
    puts("PASS native TX error/timeout recovery, fixed lifetime, tick wrap and cancellation");
}

static void test_native_forward_capacity_and_arbitration(void)
{
    CAN_TxHeaderTypeDef native = {.ExtId = 0x100U, .IDE = CAN_ID_EXT, .DLC = 3U};
    CAN_TxHeaderTypeDef telemetry = {.StdId = 0x101U, .DLC = 8U};
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    CAN_BusDiagnostics_t diag;
    reset();
    for (unsigned n = 0U; n < CAN_TX_QUEUE_LENGTH; n++)
        assert(can_transport_send(&hcan1, &telemetry, (uint8_t[8]){0}) == HAL_OK);
    for (unsigned n = 0U; n < CAN_FORWARD_QUEUE_LENGTH; n++)
        assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){(uint8_t)n}) == HAL_OK);
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){0}) == HAL_BUSY);
    assert(can_transport_send_reply(&hcan1, &ack, (uint8_t[8]){0}) == HAL_OK);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.forward_overflow == 1U && diag.forward_pending == 32U && diag.tx_failures == 1U);
    drain();
    assert(sent_count[0] == 97U && sent[0][0].header.StdId == 0x102U);
    for (unsigned n = 0U; n < CAN_FORWARD_QUEUE_LENGTH; n++)
        assert(sent[0][n + 1U].data[0] == n && sent[0][n + 1U].header.IDE == CAN_ID_EXT);

    reset();
    assert(can_transport_forward(&hcan1, &native, (uint8_t[8]){1U}) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    /* An in-flight native frame must not hide the unsent telemetry head. */
    assert(can_transport_send_latest(&hcan1, &telemetry, (uint8_t[8]){1U}, 1U) == HAL_OK);
    assert(can_transport_send_latest(&hcan1, &telemetry, (uint8_t[8]){2U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_ALST0;
    can_transport_service();
    drain();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.tx_pending == 0U && diag.forward_pending == 0U && diag.tx_failures == 0U);
    assert(diag.arbitration_lost == 1U && diag.latest_replaced == 1U && sent_count[0] == 3U);
    assert(sent[0][2].data[0] == 2U);
    puts("PASS independent native capacity, overflow accounting, arbitration retry and snapshot coalescing");
}

static void test_error_capture_from_receive_irqs(void)
{
    void CAN1_RX0_IRQHandler(void);
    void CAN2_RX0_IRQHandler(void);
    void CAN1_SCE_IRQHandler(void);
    void CAN2_SCE_IRQHandler(void);
    CAN_BusDiagnostics_t diag;
    reset();
    for (unsigned bus = 0U; bus < 2U; bus++) {
        CAN_HandleTypeDef *handle = bus ? &hcan2 : &hcan1;
        tick = 77U + bus;
        can_registers[bus].MSR = CAN_MSR_ERRI;
        can_registers[bus].ESR = 0x00400040U;
        if (bus) CAN2_RX0_IRQHandler(); else CAN1_RX0_IRQHandler();
        /* A stale SCE and ordinary RX interrupt must not duplicate/erase it. */
        if (bus) { CAN2_SCE_IRQHandler(); CAN2_RX0_IRQHandler(); }
        else { CAN1_SCE_IRQHandler(); CAN1_RX0_IRQHandler(); }
        can_transport_get_diagnostics(handle, &diag);
        assert(diag.error_events == 1U && diag.lec_events[4] == 1U);
        assert(diag.last_error_esr == 0x00400040U && diag.last_error_tick == 77U + bus);
        assert(diag.esr == 0x00400000U && diag.format_version == 3U);
        /* State-only notifications have LEC=0 but still require preservation. */
        can_registers[bus].MSR = CAN_MSR_ERRI;
        can_registers[bus].ESR = CAN_ESR_EWGF;
        if (bus) CAN2_SCE_IRQHandler(); else CAN1_SCE_IRQHandler();
        can_transport_get_diagnostics(handle, &diag);
        assert(diag.error_events == 2U && diag.error_warning_events == 1U && diag.lec_events[4] == 1U);
    }
    puts("PASS both RX0 handlers preserve LEC before HAL; no duplicate stale IRQ or normal RX snapshots");
}
