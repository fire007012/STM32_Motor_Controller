CAN_HandleTypeDef hcan1 = {0U}, hcan2 = {1U};
uint32_t test_primask;
static unsigned free_slots, sent_count, task_cycle, stop_count, range_updates;
static CAN_TxFrame_t sent[512];
static jmp_buf task_done;

HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *bus)
{ (void)bus; return HAL_CAN_STATE_LISTENING; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *bus)
{ return bus == &hcan1 ? free_slots : 0U; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *bus, uint32_t mailboxes)
{ (void)bus; (void)mailboxes; return 0U; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *bus, uint32_t mailboxes)
{ (void)bus; (void)mailboxes; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *bus,
    const CAN_TxHeaderTypeDef *header, const uint8_t data[8], uint32_t *mailbox)
{
    assert(bus == &hcan1 && free_slots && test_primask);
    assert(sent_count < sizeof(sent) / sizeof(sent[0]));
    free_slots--;
    sent[sent_count].header = last_header = *header;
    memcpy(sent[sent_count++].data, data, 8U);
    memcpy(last_can_data, data, 8U);
    *mailbox = CAN_TX_MAILBOX0;
    return HAL_OK;
}
HAL_StatusTypeDef motor_stop_all(void)
{ stop_count++; return HAL_OK; }

void motor_update_distance(const distance_sensor_sample_t *sample)
{ assert(sample->sensor_id < DISTANCE_SENSOR_COUNT); range_updates++; }

static void fill_queue(unsigned count)
{
    CAN_TxHeaderTypeDef filler = {.StdId = 0x101U, .DLC = 8U};
    uint8_t data[8] = {0};
    free_slots = 0U;
    for (unsigned i = 0U; i < count; i++)
        assert(can_transport_send(&hcan1, &filler, data) == HAL_OK);
}
static void drain(void)
{
    for (unsigned i = 0U; i < 70U; i++) {
        free_slots = 3U;
        can_transport_service();
    }
    free_slots = 0U;
    assert(test_primask == 0U);
}
static void expect_report(unsigned index, uint32_t id, uint8_t sensor,
                          uint8_t status, uint8_t sequence)
{
    assert(index < sent_count);
    assert(sent[index].header.StdId == id && sent[index].header.IDE == CAN_ID_STD);
    assert(sent[index].header.RTR == CAN_RTR_DATA && sent[index].header.DLC == 8U);
    assert(sent[index].data[0] == 1U && sent[index].data[1] == sensor);
    assert(sent[index].data[6] == status && sent[index].data[7] == sequence);
}

/* TASK FUNCTIONS */
void osDelay(uint32_t ms)
{
    assert(ms == DISTANCE_POLL_PERIOD_MS); /* Retry is bounded and still yields to other tasks. */
    switch (task_cycle++) {
    case 0U:
        /* Initialization failure has consumed the driver's one-shot flags,
         * but both reports must survive a full transport queue. */
        assert(!sensors[3].updated && !sensors[3].diagnostic_pending);
        assert(can_transport_get_failure_count(&hcan1) == 2U);
        drain();
        assert(sent_count == 64U);
        fill_queue(63U);
        break;
    case 1U:
        /* Only one slot is available: diagnostic precedes periodic distance. */
        assert(can_transport_get_failure_count(&hcan1) == 3U);
        drain();
        assert(sent_count == 128U);
        expect_report(127U, 0x111U, 3U, 8U, 1U);
        assert(last_can_data[2] == 1U && last_can_data[3] == 1U);
        assert(last_can_data[4] == 255U && last_can_data[5] == 255U);
        break;
    case 2U:
        drain();
        assert(sent_count == 129U);
        expect_report(128U, 0x110U, 3U, 8U, 1U);
        assert(last_can_data[2] == 255U && last_can_data[3] == 255U);
        break;
    case 3U:
        drain();
        assert(sent_count == 129U); /* Accepted reports are not retried. */
        can_transport_init(&hcan1, &hcan2);
        fill_queue(64U);
        for (unsigned i = 0U; i < DISTANCE_SENSOR_COUNT; i++) sensors[i].initialized = (i == 4U);
        poll_index = 4U;
        tick += 1000U;
        break;
    case 4U:
        assert(sensors[4].sample.status == DISTANCE_SENSOR_STATUS_TIMEOUT);
        assert(can_transport_get_failure_count(&hcan1) == 2U);
        drain();
        assert(sent_count == 193U);
        break;
    case 5U:
        drain();
        assert(sent_count == 195U);
        expect_report(193U, 0x111U, 4U, 4U, 1U);
        assert(sent[193U].data[2] == 3U);
        expect_report(194U, 0x110U, 4U, 4U, 1U);
        break;
    case 6U:
        drain();
        assert(sent_count == 195U); /* Persistent timeout does not flood CAN. */
        fill_queue(64U);
        memset(results, 0, sizeof(results));
        results[0] = 9U;
        results[2] = 1U;
        results[13] = 1U;
        results[14] = 244U; /* 500 mm raw. */
        ready_status = 0U;
        break;
    case 7U:
        assert(sensors[4].sample.sequence == 2U);
        results[2] = 2U;
        results[13] = 3U;
        results[14] = 32U; /* 800 mm raw. */
        break;
    case 8U:
        assert(sensors[4].sample.sequence == 3U);
        drain();
        assert(sent_count == 259U);
        ready_status = 1U; /* Retry newest pending sample without another result. */
        break;
    case 9U:
        drain();
        assert(sent_count == 260U);
        expect_report(259U, 0x110U, 4U, 1U, 3U);
        assert(last_can_data[2] == (uint8_t)sensors[4].sample.distance_mm);
        assert(last_can_data[3] == (uint8_t)(sensors[4].sample.distance_mm >> 8));
        /* Emergency handling must continue while reports cannot be queued. */
        fill_queue(64U);
        sensors[4].initialized = 0U;
        sensors[0].initialized = 1U;
        results[13] = 0U;
        results[14] = 100U;
        ready_status = 0U;
        break;
    case 10U:
        assert(stop_count == 0U && range_updates > 0U);
        assert(sensors[0].sample.status == DISTANCE_SENSOR_STATUS_VALID);
        sensors[0].initialized = 0U;
        sensors[DIST_SENSOR_REAR].initialized = 1U;
        break;
    case 11U:
        assert(stop_count == 0U);
        assert(sensors[DIST_SENSOR_REAR].sample.status == DISTANCE_SENSOR_STATUS_VALID);
        assert(can_transport_get_failure_count(&hcan1) > 0U);
        break;
    case 12U:
        /* A persistent rear obstacle must request another stop despite CAN congestion. */
        assert(stop_count == 0U);
        assert(test_primask == 0U);
        longjmp(task_done, 1);
    default:
        assert(0);
    }
    /* This fixture disables sensors artificially to isolate queue behavior;
     * recovery and error heartbeats are exercised by the sensor fixture. */
    for (unsigned i = 0U; i < DISTANCE_SENSOR_COUNT; i++) {
        sensors[i].last_recovery_tick = tick;
        if (!sensors[i].sample.valid) sensors[i].sample.timestamp_ms = tick;
    }
}
int main(void)
{
    failing_address = 0x33U;
    ready_status = 1U;
    can_transport_init(&hcan1, &hcan2);
    assert(distance_sensor_init() == HAL_ERROR);
    assert(CAN1_SendDistance(NULL) == HAL_ERROR);
    assert(CAN1_SendSensorDiag(NULL) == HAL_ERROR);
    fill_queue(64U);
    if (setjmp(task_done) == 0) StartSensorTask(NULL);
    puts("PASS sensor task: init/timeout retry, diagnostic priority, no duplicates, latest sample and front/rear raw ranges without emergency under full CAN queue");
    return 0;
}
