/* Real transport with the existing fake HAL. These are behavioral congestion
 * tests: mailbox submission/completion and task/ISR producers are independent. */
static void test_latest_capacity_and_priority(void)
{
    CAN_TxHeaderTypeDef header = {.StdId = 0x110U, .DLC = 8U};
    CAN_TxHeaderTypeDef native = {.ExtId = 0x100U, .IDE = CAN_ID_EXT, .DLC = 5U};
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    CAN_BusDiagnostics_t diag;
    const unsigned limit = CAN_TX_QUEUE_LENGTH - CAN_TX_RESERVED_SLOTS;
    reset();
    for (unsigned key = 0U; key < limit; key++) {
        uint8_t data[8] = {(uint8_t)key, 1U};
        assert(can_transport_send_latest(&hcan1, &header, data, (uint16_t)key) == HAL_OK);
    }
    /* Only new streams are refused; updates consume no additional capacity. */
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){99U}, 99U) == HAL_BUSY);
    assert(can_transport_get_failure_count(&hcan1) == 0U);
    for (unsigned i = 0U; i < CAN_TX_RESERVED_SLOTS; i++) {
        uint8_t data[8] = {0x35U, 0U, 0U, (uint8_t)i, 0x6BU};
        assert(can_transport_send(&hcan1, &native, data) == HAL_OK);
    }
    test_primask = 1U;
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){0U, 9U}, 0U) == HAL_OK);
    assert(test_primask == 1U);
    test_primask = 0U;
    for (unsigned i = 0U; i < CAN_REPLY_QUEUE_LENGTH; i++) {
        uint8_t data[8] = {(uint8_t)i};
        assert(can_transport_send_reply(&hcan1, &ack, data) == HAL_OK);
    }
    assert(can_transport_send(&hcan1, &native, (uint8_t[8]){0}) == HAL_BUSY);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.tx_pending == 64U && diag.tx_high_water == 64U);
    assert(diag.latest_backpressure == 1U && diag.latest_replaced == 1U);
    assert(diag.queue_full == 1U && diag.reply_pending == 32U);
    assert(diag.tx_failures == 1U);
    drain();
    assert(sent_count[0] == CAN_TX_QUEUE_LENGTH + CAN_REPLY_QUEUE_LENGTH);
    for (unsigned i = 0U; i < CAN_REPLY_QUEUE_LENGTH; i++) {
        assert(sent[0][i].header.StdId == 0x102U && sent[0][i].data[0] == i);
    }
    assert(sent[0][CAN_REPLY_QUEUE_LENGTH].data[1] == 9U);
    for (unsigned i = 0U; i < CAN_TX_RESERVED_SLOTS; i++) {
        unsigned pos = CAN_REPLY_QUEUE_LENGTH + limit + i;
        assert(sent[0][pos].header.IDE == CAN_ID_EXT && sent[0][pos].data[3] == i);
    }

    /* Atomic preflight: a rejected batch cannot partially refresh one type. */
    reset();
    for (unsigned key = 0U; key < limit - 1U; key++)
        assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, (uint16_t)key) == HAL_OK);
    CAN_TxFrame_t batch[3] = {{.header = header}, {.header = header}, {.header = header}};
    uint16_t keys[3] = {0U, 100U, 101U};
    batch[0].data[0] = 9U;
    assert(can_transport_send_latest_batch(&hcan1, batch, keys, 3U) == HAL_BUSY);
    drain();
    assert(sent_count[0] == limit - 1U && sent[0][0].data[0] == 1U);
    keys[1] = keys[0];
    assert(can_transport_send_latest_batch(&hcan1, batch, keys, 2U) == HAL_ERROR);
    assert(can_transport_send_latest(&hcan2, &header, (uint8_t[8]){0}, 0U) == HAL_ERROR);
    puts("PASS latest snapshots: 56-slot bound, 8 native slots, 32 independent replies, FIFO order and atomic backpressure");
}

static void test_latest_in_flight_and_recovery(void)
{
    CAN_TxHeaderTypeDef header = {.StdId = 0x101U, .DLC = 8U};
    CAN_TxHeaderTypeDef ack = {.StdId = 0x102U, .DLC = 8U};
    CAN_BusDiagnostics_t diag;
    reset();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(sent_count[0] == 1U);
    /* The hardware already has value 1. Keep exactly one newest successor. */
    for (uint8_t value = 2U; value <= 20U; value++)
        assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){value}, 1U) == HAL_OK);
    assert(can_transport_send_reply(&hcan1, &ack, (uint8_t[8]){9U}) == HAL_OK);
    can_transport_service();
    assert(sent_count[0] == 1U); /* Enqueue is not TXOK; mailbox still pending. */
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.tx_ok == 0U && diag.tx_pending == 2U && diag.latest_replaced == 18U);
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(sent_count[0] == 2U && sent[0][1].header.StdId == 0x102U);
    drain();
    assert(sent_count[0] == 3U && sent[0][0].data[0] == 1U && sent[0][2].data[0] == 20U);

    reset();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){2U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_ALST0;
    can_transport_service();
    drain();
    assert(sent_count[0] == 2U && sent[0][1].data[0] == 2U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.arbitration_lost == 1U && diag.tx_failures == 0U);

    /* Same numeric ID/key in different frame formats must not collide. */
    reset();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, 1U) == HAL_OK);
    CAN_TxHeaderTypeDef extended = {.ExtId = 0x101U, .IDE = CAN_ID_EXT, .DLC = 8U};
    assert(can_transport_send_latest(&hcan1, &extended, (uint8_t[8]){2U}, 1U) == HAL_OK);
    drain();
    assert(sent_count[0] == 2U && sent[0][1].header.IDE == CAN_ID_EXT);

    /* Real HAL failures stay visible; replacing during backoff never replays
     * the old payload, and temporary mailbox saturation is not an error. */
    reset();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, 1U) == HAL_OK);
    fail_next[0] = 1U;
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(can_transport_get_failure_count(&hcan1) == 1U);
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){2U}, 1U) == HAL_OK);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.hal_submit_errors == 1U && diag.queue_full == 0U && diag.latest_replaced == 1U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 1U && sent[0][0].data[0] == 2U);
    busy_next[0] = 1U;
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){3U}, 1U) == HAL_OK);
    can_transport_service();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.hal_submit_busy == 1U && can_transport_get_failure_count(&hcan1) == 2U);
    tick += CAN_ROS_TX_BACKOFF_MS;
    drain();
    assert(sent_count[0] == 2U && sent[0][1].data[0] == 3U);

    reset();
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){1U}, 1U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    free_mailboxes[0] = 3U;
    can_registers[0].TSR = CAN_TSR_TERR0;
    can_transport_service();
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.tx_error == 1U && diag.tx_failures == 1U && diag.hal_submit_errors == 0U);
    puts("PASS latest snapshots: in-flight immutability, next-service reply priority, format keys and distinct HAL/hardware failures");
}

static unsigned latest_callback_count;
static void latest_complete(uint32_t token)
{
    assert(token == 123U && test_primask == 1U);
    latest_callback_count++;
}

static void test_latest_ring_and_callbacks(void)
{
    CAN_TxHeaderTypeDef header = {.StdId = 0x110U, .DLC = 8U};
    CAN_TxFrame_t native = {.header = {.ExtId = 0x100U, .IDE = CAN_ID_EXT, .DLC = 5U}};
    CAN_BusDiagnostics_t diag;
    reset();
    latest_callback_count = 0U;
    for (unsigned round = 0U; round < 100U; round++) {
        uint8_t data[8] = {(uint8_t)round};
        assert(can_transport_send_latest(&hcan1, &header, data, 0U) == HAL_OK);
        assert(can_transport_send_latest(&hcan1, &header, data, 1U) == HAL_OK);
        data[1] = 7U;
        assert(can_transport_send_latest(&hcan1, &header, data, 0U) == HAL_OK);
        assert(can_transport_send_batch_notify(&hcan1, &native, 1U, latest_complete, 123U) == HAL_OK);
        drain();
        unsigned pos = round * 3U;
        assert(sent[0][pos].data[0] == round && sent[0][pos].data[1] == 7U);
        assert(sent[0][pos + 1U].data[1] == 0U);
        assert(sent[0][pos + 2U].header.IDE == CAN_ID_EXT);
        assert(latest_callback_count == round + 1U);
    }
    assert(can_transport_send_batch_notify(&hcan1, &native, 1U, latest_complete, 123U) == HAL_OK);
    free_mailboxes[0] = 3U;
    can_transport_service();
    assert(latest_callback_count == 100U); /* Submission alone is not completion. */
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){8U}, 0U) == HAL_OK);
    can_transport_cancel(&hcan1);
    drain();
    assert(latest_callback_count == 100U && sent_count[0] == 301U);
    can_transport_get_diagnostics(&hcan1, &diag);
    assert(diag.canceled_frames == 2U && diag.tx_pending == 0U);
    assert(can_transport_send_latest(&hcan1, &header, (uint8_t[8]){9U}, 0U) == HAL_OK);
    drain();
    assert(sent_count[0] == 302U && sent[0][301].data[0] == 9U);
    for (unsigned bus = 0U; bus < 2U; bus++) {
        reset();
        latest_callback_count = 0U;
        CAN_HandleTypeDef *handle = bus == 0U ? &hcan1 : &hcan2;
        assert(can_transport_send_batch_notify(handle, &native, 1U, latest_complete, 123U) == HAL_OK);
        free_mailboxes[bus] = 3U;
        can_transport_service();
        assert(latest_callback_count == 0U);
        free_mailboxes[bus] = 3U;
        can_registers[bus].TSR = CAN_TSR_TERR0;
        can_transport_poll_completions(handle);
        assert(latest_callback_count == 0U && can_transport_get_failure_count(handle) == 1U);
    }
    puts("PASS latest snapshots: ring wrap across 300 transmissions, callback timing, cancellation and fresh reuse");
}
