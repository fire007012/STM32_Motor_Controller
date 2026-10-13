static void prepare(uint16_t distance, uint8_t status)
{
    memset(results, 0, sizeof(results));
    results[0] = status;
    results[2] = 1U;
    results[13] = (uint8_t)(distance >> 8);
    results[14] = (uint8_t)distance;
    range_read_failure = ready_status = 0U;
}
static void reset_bus(uint8_t fail)
{
    tick = high_pins = initialized_count = range_reads = 0U;
    memset(assigned_pins, 0, sizeof(assigned_pins));
    failing_address = fail;
}
int main(void)
{
    distance_sensor_sample_t sample;
    distance_sensor_diag_t diag;
    reset_bus(0U);
    assert(distance_sensor_init() == HAL_OK);
    assert(initialized_count == 5U && high_pins == DISTANCE_SENSOR_XSHUT_PINS);
    for (unsigned i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        assert(sensors[i].initialized && sensors[i].address == 0x30U+i);
        assert(assigned_pins[sensors[i].xshut_pin] == 0x30U+i);
    }

    /* Round-robin reaches front, original sides, and both new IDs. */
    prepare(500U, 9U);
    for (unsigned i = 0; i < 5U; i++) {
        tick += 10U;
        distance_sensor_poll();
        assert(range_reads == i+1U);
        assert(distance_sensor_take_updated(i, &sample));
        assert(sample.sensor_id == i && sample.valid && sample.distance_mm == 491U);
        assert(!distance_sensor_take_updated(i, &sample));
        CAN1_SendDistance(&sample);
        assert(last_header.StdId == 0x110U && last_header.IDE == CAN_ID_STD && last_header.DLC == 8U);
        assert(last_can_data[0] == 1U && last_can_data[1] == i);
        assert(last_can_data[2] == 0xEBU && last_can_data[3] == 1U);
        assert(last_can_data[4] == 255U && last_can_data[5] == 255U);
        assert(last_can_data[6] == 1U && last_can_data[7] == 1U);
    }
    assert(!distance_sensor_take_updated(5U, &sample));
    assert(!distance_sensor_take_updated(0U, NULL));

    /* Close targets on upper sensors do not request a local chassis stop. */
    prepare(100U, 9U);
    for (unsigned i = 0; i < 5U; i++) {
        sensors[i].filter_count = sensors[i].filter_index = 0U;
        vl53_process_ready_sample(&sensors[i]);
        assert(distance_sensor_take_updated(i, &sample) && sample.valid);
        assert(!!(sample.status & DISTANCE_SENSOR_STATUS_EMERGENCY) == (i < 3U));
        assert(distance_sensor_take_emergency(&sample) == (i < 3U));
        assert(!distance_sensor_take_emergency(&sample));
    }

    /* An invalid current range cannot borrow an old valid median. */
    prepare(0U, 9U);
    vl53_process_ready_sample(&sensors[3]);
    assert(distance_sensor_take_updated(3U, &sample));
    assert(!sample.valid && sample.distance_mm == 0xFFFFU && sample.status == 2U);
    assert(sensors[3].filter_count == 1U);
    assert(distance_sensor_take_diagnostic(&diag) && diag.sensor_id == 3U && diag.error_code == 2U);

    prepare(500U, 7U);
    vl53_process_ready_sample(&sensors[4]);
    assert(distance_sensor_take_updated(4U, &sample));
    assert(!sample.valid && sample.status == 0x10U && sample.distance_mm == 0xFFFFU);
    assert(distance_sensor_take_diagnostic(&diag) && diag.sensor_id == 4U && diag.error_code == 7U);
    range_read_failure = 1U;
    vl53_process_ready_sample(&sensors[4]);
    assert(distance_sensor_take_updated(4U, &sample) && sample.status == 8U);
    assert(distance_sensor_take_diagnostic(&diag) && diag.sensor_id == 4U && diag.error_code == 1U);

    /* No new result becomes one timeout event, not a flood. */
    range_read_failure = 0U;
    ready_status = 1U;
    tick += 201U;
    poll_index = 4U;
    distance_sensor_poll();
    assert(distance_sensor_take_updated(4U, &sample) && sample.status == 4U);
    assert(distance_sensor_take_diagnostic(&diag) && diag.sensor_id == 4U && diag.error_code == 3U);
    poll_index = 4U;
    distance_sensor_poll();
    assert(!distance_sensor_take_updated(4U, &sample));

    /* A failed sensor is reset/isolated; later devices still initialize. */
    reset_bus(0x33U);
    assert(distance_sensor_init() == HAL_ERROR && initialized_count == 5U);
    assert(!sensors[3].initialized && sensors[4].initialized);
    assert(!(high_pins & DISTANCE_SENSOR_XSHUT_ARM_UPPER_LEFT_Pin));
    assert(high_pins & DISTANCE_SENSOR_XSHUT_ARM_UPPER_RIGHT_Pin);
    assert(distance_sensor_take_updated(3U, &sample) && sample.sensor_id == 3U && sample.status == 8U);
    assert(distance_sensor_take_diagnostic(&diag) && diag.sensor_id == 3U);
    prepare(500U, 9U);
    poll_index = 3U;
    distance_sensor_poll();
    assert(distance_sensor_take_updated(4U, &sample) && sample.valid);
    return 0;
}
