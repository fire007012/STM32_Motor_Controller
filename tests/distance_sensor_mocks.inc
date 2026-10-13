GPIO_TypeDef gpioe;
static uint32_t tick;
static uint16_t high_pins;
static uint8_t assigned_pins[256];
static uint8_t failing_address;
static unsigned initialized_count, range_reads;
static uint8_t results[17], ready_status, range_read_failure;
static CAN_TxHeaderTypeDef last_header;
static uint8_t last_can_data[8];

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOE);
    if (state) high_pins |= pin;
    else {
        high_pins &= (uint16_t)~pin;
        assigned_pins[pin] = 0U;
    }
}
static void i2c1_configure(void) { i2c1_initialized = 1U; }
static HAL_StatusTypeDef vl53_read_u16(uint8_t address, uint16_t reg, uint16_t *value)
{
    if (address == 0x29U) {
        unsigned default_devices = 0U;
        for (unsigned i = 0; i < sizeof(assigned_pins); i++)
            if ((high_pins & i) == i && i != 0U && (i & (i - 1U)) == 0U && !assigned_pins[i])
                default_devices++;
        assert(default_devices == 1U); /* All other unassigned sensors stay in reset. */
        assert(reg == VL53L1X_MODEL_ID_REG);
        *value = VL53L1X_MODEL_ID;
    } else {
        assert(reg == VL53L1X_FAST_OSC_FREQ_REG || reg == VL53L1X_OSC_CALIBRATE_REG);
        *value = 100U;
    }
    return HAL_OK;
}
static HAL_StatusTypeDef vl53_write_u8(uint8_t address, uint16_t reg, uint8_t value)
{
    if (reg == VL53L1X_I2C_ADDRESS_REG) {
        assert(address == 0x29U);
        assert(value == 0x30U + initialized_count);
        for (unsigned i = 0; i < sizeof(assigned_pins); i++)
            if ((high_pins & i) == i && i && !(i & (i-1U)) && !assigned_pins[i])
                assigned_pins[i] = value;
        initialized_count++;
    }
    return HAL_OK;
}
static HAL_StatusTypeDef vl53_read_u8(uint8_t address, uint16_t reg, uint8_t *value)
{
    (void)address;
    *value = reg == VL53L1X_BOOT_STATUS_REG ? 1U : ready_status;
    return HAL_OK;
}
static HAL_StatusTypeDef vl53_apply_ranging_config(vl53l1x_sensor_t *sensor)
{ return sensor->address == failing_address ? HAL_ERROR : HAL_OK; }
static HAL_StatusTypeDef vl53_start_continuous(vl53l1x_sensor_t *sensor)
{ (void)sensor; return HAL_OK; }
static HAL_StatusTypeDef vl53_read(uint8_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    assert(address >= 0x30U && address <= 0x35U);
    assert(reg == VL53L1X_RESULT_RANGE_STATUS_REG && length == sizeof(results));
    if (range_read_failure) return HAL_ERROR;
    memcpy(data, results, length);
    range_reads++;
    return HAL_OK;
}
static void vl53_update_dss(vl53l1x_sensor_t *sensor, const uint8_t data[17])
{ (void)sensor; (void)data; }
static void vl53_setup_manual_calibration(vl53l1x_sensor_t *sensor)
{ sensor->calibrated = 1U; }
static HAL_StatusTypeDef CAN1_TrySend(const CAN_TxHeaderTypeDef *header, uint8_t data[8])
{ last_header = *header; memcpy(last_can_data, data, 8U); return HAL_OK; }

