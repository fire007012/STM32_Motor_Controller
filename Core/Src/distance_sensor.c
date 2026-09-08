#include "distance_sensor.h"

#include <string.h>

#define VL53L1X_DEFAULT_ADDR              0x29U
#define VL53L1X_ADDR_FRONT                0x30U
#define VL53L1X_ADDR_LEFT                 0x31U
#define VL53L1X_ADDR_RIGHT                0x32U

#define VL53L1X_MODEL_ID_REG              0x010FU
#define VL53L1X_MODEL_ID                  0xEACCU
#define VL53L1X_I2C_ADDRESS_REG           0x0001U
#define VL53L1X_SOFT_RESET_REG            0x0000U
#define VL53L1X_BOOT_STATUS_REG           0x00E5U
#define VL53L1X_EXTSUP_CONFIG_REG         0x002EU
#define VL53L1X_FAST_OSC_FREQ_REG         0x0006U
#define VL53L1X_OSC_CALIBRATE_REG         0x00DEU
#define VL53L1X_GPIO_STATUS_REG           0x0031U
#define VL53L1X_INTERRUPT_CLEAR_REG       0x0086U
#define VL53L1X_MODE_START_REG            0x0087U
#define VL53L1X_INTERMEASUREMENT_REG      0x006CU
#define VL53L1X_RESULT_RANGE_STATUS_REG   0x0089U

#define VL53L1X_I2C_TIMEOUT_MS            10U
#define VL53L1X_BOOT_TIMEOUT_MS           100U
#define VL53L1X_SAMPLE_TIMEOUT_MS         200U
#define VL53L1X_INTERMEASUREMENT_MS       50U
#define VL53L1X_MIN_DISTANCE_MM           50U
#define VL53L1X_MAX_DISTANCE_MM           3000U
#define VL53L1X_EMERGENCY_DISTANCE_MM     300U
#define VL53L1X_TIMING_BUDGET_US          33000U
#define VL53L1X_TIMING_GUARD_US           4528U
#define VL53L1X_TARGET_RATE               0x0A00U
#define VL53L1X_FILTER_SIZE                3U

#define VL53L1X_REG_RESULT_STATUS         0U
#define VL53L1X_REG_RESULT_STREAM_COUNT   2U
#define VL53L1X_REG_RESULT_SPADS_HI       3U
#define VL53L1X_REG_RESULT_AMBIENT_HI     7U
#define VL53L1X_REG_RESULT_RANGE_HI       13U
#define VL53L1X_REG_RESULT_PEAK_HI        15U

typedef struct {
    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;
    uint8_t address;
    uint8_t initialized;
    uint8_t calibrated;
    uint8_t sequence;
    uint8_t consecutive_error_count;
    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;
    uint16_t filter_mm[VL53L1X_FILTER_SIZE];
    uint8_t filter_count;
    uint8_t filter_index;
    uint32_t last_sample_tick;
    distance_sensor_sample_t sample;
    uint8_t updated;
    uint8_t diagnostic_pending;
    uint8_t diagnostic_error_code;
} vl53l1x_sensor_t;

static vl53l1x_sensor_t sensors[DISTANCE_SENSOR_COUNT] = {
    {GPIOE, GPIO_PIN_0, VL53L1X_ADDR_FRONT, 0U},
    {GPIOE, GPIO_PIN_1, VL53L1X_ADDR_LEFT, 0U},
    {GPIOE, GPIO_PIN_2, VL53L1X_ADDR_RIGHT, 0U}
};
static uint8_t i2c1_initialized = 0U;
static uint8_t poll_index = 0U;
static uint8_t emergency_pending = 0U;
static distance_sensor_sample_t emergency_sample;

static uint16_t read_be16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static void i2c1_configure(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;
    I2C1->CR2 = 36U; /* APB1 is 36 MHz. */
    I2C1->OAR1 = I2C_OAR1_ADD0;
    I2C1->CCR = I2C_CCR_FS | 30U; /* 36 MHz / (3 * 400 kHz). */
    I2C1->TRISE = 11U; /* 300 ns rise time at 36 MHz. */
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
    i2c1_initialized = 1U;
}

static void i2c1_abort(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;
    I2C1->CR2 = 36U;
    I2C1->OAR1 = I2C_OAR1_ADD0;
    I2C1->CCR = I2C_CCR_FS | 30U;
    I2C1->TRISE = 11U;
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
}

static HAL_StatusTypeDef i2c1_wait_sr1(uint32_t flag)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t errors = I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT;

    while ((I2C1->SR1 & flag) == 0U) {
        if ((I2C1->SR1 & errors) != 0U || (HAL_GetTick() - start_tick) >= VL53L1X_I2C_TIMEOUT_MS) {
            i2c1_abort();
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef i2c1_wait_idle(void)
{
    uint32_t start_tick = HAL_GetTick();

    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
        if ((HAL_GetTick() - start_tick) >= VL53L1X_I2C_TIMEOUT_MS) {
            i2c1_abort();
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

static void i2c1_clear_addr(void)
{
    volatile uint32_t discard;

    discard = I2C1->SR1;
    discard = I2C1->SR2;
    (void)discard;
}

static HAL_StatusTypeDef i2c1_start(uint8_t address, uint8_t read)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (i2c1_wait_sr1(I2C_SR1_SB) != HAL_OK) {
        return HAL_ERROR;
    }

    I2C1->DR = (uint8_t)((address << 1) | (read != 0U ? 1U : 0U));
    return i2c1_wait_sr1(I2C_SR1_ADDR);
}

static HAL_StatusTypeDef vl53_write(uint8_t address, uint16_t reg, const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == NULL) || (length == 0U) || (i2c1_wait_idle() != HAL_OK) ||
        (i2c1_start(address, 0U) != HAL_OK)) {
        return HAL_ERROR;
    }

    i2c1_clear_addr();
    I2C1->DR = (uint8_t)(reg >> 8);
    if (i2c1_wait_sr1(I2C_SR1_TXE) != HAL_OK) {
        return HAL_ERROR;
    }
    I2C1->DR = (uint8_t)reg;
    if (i2c1_wait_sr1(I2C_SR1_TXE) != HAL_OK) {
        return HAL_ERROR;
    }

    for (i = 0U; i < length; i++) {
        I2C1->DR = data[i];
        if (i2c1_wait_sr1(I2C_SR1_TXE) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    if (i2c1_wait_sr1(I2C_SR1_BTF) != HAL_OK) {
        return HAL_ERROR;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    return HAL_OK;
}

static HAL_StatusTypeDef vl53_read(uint8_t address, uint16_t reg, uint8_t *data, uint16_t length)
{
    uint16_t remaining = length;

    if ((data == NULL) || (length == 0U) || (i2c1_wait_idle() != HAL_OK) ||
        (i2c1_start(address, 0U) != HAL_OK)) {
        return HAL_ERROR;
    }

    i2c1_clear_addr();
    I2C1->DR = (uint8_t)(reg >> 8);
    if (i2c1_wait_sr1(I2C_SR1_TXE) != HAL_OK) {
        return HAL_ERROR;
    }
    I2C1->DR = (uint8_t)reg;
    if (i2c1_wait_sr1(I2C_SR1_BTF) != HAL_OK) {
        return HAL_ERROR;
    }

    if (i2c1_start(address, 1U) != HAL_OK) {
        return HAL_ERROR;
    }

    if (length == 1U) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c1_clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;
        if (i2c1_wait_sr1(I2C_SR1_RXNE) != HAL_OK) {
            return HAL_ERROR;
        }
        data[0] = (uint8_t)I2C1->DR;
        I2C1->CR1 |= I2C_CR1_ACK;
        return HAL_OK;
    }

    if (length == 2U) {
        I2C1->CR1 |= I2C_CR1_POS;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c1_clear_addr();
        if (i2c1_wait_sr1(I2C_SR1_BTF) != HAL_OK) {
            return HAL_ERROR;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)I2C1->DR;
        data[1] = (uint8_t)I2C1->DR;
        I2C1->CR1 &= ~I2C_CR1_POS;
        I2C1->CR1 |= I2C_CR1_ACK;
        return HAL_OK;
    }

    I2C1->CR1 |= I2C_CR1_ACK;
    i2c1_clear_addr();
    while (remaining > 3U) {
        if (i2c1_wait_sr1(I2C_SR1_RXNE) != HAL_OK) {
            return HAL_ERROR;
        }
        *data++ = (uint8_t)I2C1->DR;
        remaining--;
    }

    if (i2c1_wait_sr1(I2C_SR1_BTF) != HAL_OK) {
        return HAL_ERROR;
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    *data++ = (uint8_t)I2C1->DR;
    remaining--;
    if (i2c1_wait_sr1(I2C_SR1_BTF) != HAL_OK) {
        return HAL_ERROR;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    *data++ = (uint8_t)I2C1->DR;
    *data = (uint8_t)I2C1->DR;
    I2C1->CR1 |= I2C_CR1_ACK;
    return HAL_OK;
}

static HAL_StatusTypeDef vl53_write_u8(uint8_t address, uint16_t reg, uint8_t value)
{
    return vl53_write(address, reg, &value, 1U);
}

static HAL_StatusTypeDef vl53_write_u16(uint8_t address, uint16_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return vl53_write(address, reg, data, sizeof(data));
}

static HAL_StatusTypeDef vl53_write_u32(uint8_t address, uint16_t reg, uint32_t value)
{
    uint8_t data[4];

    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
    return vl53_write(address, reg, data, sizeof(data));
}

static HAL_StatusTypeDef vl53_read_u8(uint8_t address, uint16_t reg, uint8_t *value)
{
    return vl53_read(address, reg, value, 1U);
}

static HAL_StatusTypeDef vl53_read_u16(uint8_t address, uint16_t reg, uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    status = vl53_read(address, reg, data, sizeof(data));
    if (status == HAL_OK) {
        *value = read_be16(data);
    }
    return status;
}

static void sensor_set_xshut(const vl53l1x_sensor_t *sensor, GPIO_PinState state)
{
    HAL_GPIO_WritePin(sensor->xshut_port, sensor->xshut_pin, state);
}

static void sensor_mark_error(vl53l1x_sensor_t *sensor, uint8_t status, uint8_t error_code)
{
    sensor->sample.sensor_id = (uint8_t)(sensor - sensors);
    sensor->sample.distance_mm = 0xFFFFU;
    sensor->sample.sigma_mm = 0xFFFFU;
    sensor->sample.timestamp_ms = HAL_GetTick();
    sensor->sample.valid = 0U;
    sensor->sample.status = status;
    sensor->sample.sequence++;
    sensor->sequence = sensor->sample.sequence;
    sensor->updated = 1U;
    sensor->diagnostic_pending = 1U;
    sensor->diagnostic_error_code = error_code;
    if (sensor->consecutive_error_count != 0xFFU) {
        sensor->consecutive_error_count++;
    }
}

static uint16_t vl53_encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte;
    uint16_t ms_byte = 0U;

    if (timeout_mclks == 0U) {
        return 0U;
    }

    ls_byte = timeout_mclks - 1U;
    while ((ls_byte & 0xFFFFFF00UL) != 0U) {
        ls_byte >>= 1;
        ms_byte++;
    }

    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFFU));
}

static uint32_t vl53_macro_period_us(const vl53l1x_sensor_t *sensor, uint8_t vcsel_period)
{
    uint32_t pll_period_us;
    uint32_t macro_period_us;
    uint8_t vcsel_period_pclks;

    if (sensor->fast_osc_frequency == 0U) {
        return 0U;
    }

    pll_period_us = (uint32_t)0x40000000UL / sensor->fast_osc_frequency;
    vcsel_period_pclks = (uint8_t)((vcsel_period + 1U) << 1);
    macro_period_us = 2304U * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;
    return macro_period_us;
}

static uint32_t vl53_timeout_us_to_mclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    if (macro_period_us == 0U) {
        return 0U;
    }

    return ((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

static HAL_StatusTypeDef vl53_set_timing_budget(vl53l1x_sensor_t *sensor, uint32_t budget_us)
{
    uint32_t range_timeout_us;
    uint32_t macro_period_us;
    uint32_t phasecal_timeout_mclks;

    if (budget_us <= VL53L1X_TIMING_GUARD_US) {
        return HAL_ERROR;
    }

    range_timeout_us = (budget_us - VL53L1X_TIMING_GUARD_US) / 2U;
    macro_period_us = vl53_macro_period_us(sensor, 0x0FU);
    if (macro_period_us == 0U) {
        return HAL_ERROR;
    }

    phasecal_timeout_mclks = vl53_timeout_us_to_mclks(1000U, macro_period_us);
    if (phasecal_timeout_mclks > 0xFFU) {
        phasecal_timeout_mclks = 0xFFU;
    }

    if ((vl53_write_u8(sensor->address, 0x004BU, (uint8_t)phasecal_timeout_mclks) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x005AU,
                         vl53_encode_timeout(vl53_timeout_us_to_mclks(1U, macro_period_us))) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x005EU,
                         vl53_encode_timeout(vl53_timeout_us_to_mclks(range_timeout_us, macro_period_us))) != HAL_OK)) {
        return HAL_ERROR;
    }

    macro_period_us = vl53_macro_period_us(sensor, 0x0DU);
    if (macro_period_us == 0U) {
        return HAL_ERROR;
    }

    return ((vl53_write_u16(sensor->address, 0x005CU,
                             vl53_encode_timeout(vl53_timeout_us_to_mclks(1U, macro_period_us))) == HAL_OK) &&
            (vl53_write_u16(sensor->address, 0x0061U,
                             vl53_encode_timeout(vl53_timeout_us_to_mclks(range_timeout_us, macro_period_us))) == HAL_OK))
               ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef vl53_apply_ranging_config(vl53l1x_sensor_t *sensor)
{
    uint8_t ext_supply;
    uint16_t outer_offset;

    if ((vl53_write_u16(sensor->address, 0x0024U, VL53L1X_TARGET_RATE) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0031U, 0x02U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0036U, 8U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0037U, 16U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0039U, 0x01U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x003EU, 0xFFU) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x003FU, 0U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0040U, 2U) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x0050U, 0U) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x0052U, 0U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0057U, 0x38U) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x0064U, 360U) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x0066U, 192U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0071U, 0x01U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x007CU, 0x01U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x007EU, 2U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0082U, 0U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0077U, 1U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0081U, 0x8BU) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x0054U, 200U << 8) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x004FU, 2U) != HAL_OK)) {
        return HAL_ERROR;
    }

    if (vl53_read_u8(sensor->address, VL53L1X_EXTSUP_CONFIG_REG, &ext_supply) != HAL_OK ||
        vl53_write_u8(sensor->address, VL53L1X_EXTSUP_CONFIG_REG, (uint8_t)(ext_supply | 0x01U)) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Long-distance preset from the ST low-power autonomous ranging mode. */
    if ((vl53_write_u8(sensor->address, 0x0060U, 0x0FU) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0063U, 0x0DU) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0069U, 0xB8U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0078U, 0x0FU) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x0079U, 0x0DU) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x007AU, 14U) != HAL_OK) ||
        (vl53_write_u8(sensor->address, 0x007BU, 14U) != HAL_OK) ||
        (vl53_set_timing_budget(sensor, VL53L1X_TIMING_BUDGET_US) != HAL_OK) ||
        (vl53_read_u16(sensor->address, 0x0022U, &outer_offset) != HAL_OK) ||
        (vl53_write_u16(sensor->address, 0x001EU, (uint16_t)(outer_offset * 4U)) != HAL_OK)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef vl53_start_continuous(vl53l1x_sensor_t *sensor)
{
    uint32_t period;

    period = (uint32_t)VL53L1X_INTERMEASUREMENT_MS * sensor->osc_calibrate_val;
    return ((vl53_write_u32(sensor->address, VL53L1X_INTERMEASUREMENT_REG, period) == HAL_OK) &&
            (vl53_write_u8(sensor->address, VL53L1X_INTERRUPT_CLEAR_REG, 0x01U) == HAL_OK) &&
            (vl53_write_u8(sensor->address, VL53L1X_MODE_START_REG, 0x40U) == HAL_OK))
               ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef vl53_init_one(vl53l1x_sensor_t *sensor)
{
    uint16_t model_id;
    uint32_t start_tick;
    uint8_t boot_status = 0U;

    sensor_set_xshut(sensor, GPIO_PIN_SET);
    HAL_Delay(2U);

    if (vl53_read_u16(VL53L1X_DEFAULT_ADDR, VL53L1X_MODEL_ID_REG, &model_id) != HAL_OK ||
        model_id != VL53L1X_MODEL_ID) {
        return HAL_ERROR;
    }

    if ((vl53_write_u8(VL53L1X_DEFAULT_ADDR, VL53L1X_SOFT_RESET_REG, 0x00U) != HAL_OK)) {
        return HAL_ERROR;
    }
    HAL_Delay(1U);
    if (vl53_write_u8(VL53L1X_DEFAULT_ADDR, VL53L1X_SOFT_RESET_REG, 0x01U) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2U);

    start_tick = HAL_GetTick();
    do {
        if (vl53_read_u8(VL53L1X_DEFAULT_ADDR, VL53L1X_BOOT_STATUS_REG, &boot_status) == HAL_OK &&
            (boot_status & 0x01U) != 0U) {
            break;
        }
        HAL_Delay(1U);
    } while ((HAL_GetTick() - start_tick) < VL53L1X_BOOT_TIMEOUT_MS);

    if ((boot_status & 0x01U) == 0U ||
        vl53_write_u8(VL53L1X_DEFAULT_ADDR, VL53L1X_I2C_ADDRESS_REG, sensor->address) != HAL_OK ||
        vl53_read_u16(sensor->address, VL53L1X_FAST_OSC_FREQ_REG, &sensor->fast_osc_frequency) != HAL_OK ||
        vl53_read_u16(sensor->address, VL53L1X_OSC_CALIBRATE_REG, &sensor->osc_calibrate_val) != HAL_OK ||
        sensor->fast_osc_frequency == 0U || sensor->osc_calibrate_val == 0U ||
        vl53_apply_ranging_config(sensor) != HAL_OK ||
        vl53_start_continuous(sensor) != HAL_OK) {
        return HAL_ERROR;
    }

    sensor->initialized = 1U;
    sensor->last_sample_tick = HAL_GetTick();
    sensor->sample.sensor_id = (uint8_t)(sensor - sensors);
    sensor->sample.distance_mm = 0xFFFFU;
    sensor->sample.sigma_mm = 0xFFFFU;
    return HAL_OK;
}

static uint16_t median_filter(vl53l1x_sensor_t *sensor, uint16_t distance_mm)
{
    uint16_t ordered[VL53L1X_FILTER_SIZE];
    uint8_t i;
    uint8_t j;
    uint16_t tmp;

    sensor->filter_mm[sensor->filter_index] = distance_mm;
    sensor->filter_index = (uint8_t)((sensor->filter_index + 1U) % VL53L1X_FILTER_SIZE);
    if (sensor->filter_count < VL53L1X_FILTER_SIZE) {
        sensor->filter_count++;
    }

    for (i = 0U; i < sensor->filter_count; i++) {
        ordered[i] = sensor->filter_mm[i];
    }
    for (i = 0U; i < sensor->filter_count; i++) {
        for (j = (uint8_t)(i + 1U); j < sensor->filter_count; j++) {
            if (ordered[j] < ordered[i]) {
                tmp = ordered[i];
                ordered[i] = ordered[j];
                ordered[j] = tmp;
            }
        }
    }

    return ordered[sensor->filter_count / 2U];
}

static void vl53_update_dss(vl53l1x_sensor_t *sensor, const uint8_t results[17])
{
    uint16_t spad_count = read_be16(&results[VL53L1X_REG_RESULT_SPADS_HI]);
    uint16_t ambient_rate = read_be16(&results[VL53L1X_REG_RESULT_AMBIENT_HI]);
    uint16_t peak_rate = read_be16(&results[VL53L1X_REG_RESULT_PEAK_HI]);
    uint32_t total_rate_per_spad;
    uint32_t required_spads;

    if (spad_count == 0U) {
        (void)vl53_write_u16(sensor->address, 0x0054U, 0x8000U);
        return;
    }

    total_rate_per_spad = (uint32_t)ambient_rate + peak_rate;
    if (total_rate_per_spad > 0xFFFFU) {
        total_rate_per_spad = 0xFFFFU;
    }
    total_rate_per_spad <<= 16;
    total_rate_per_spad /= spad_count;

    if (total_rate_per_spad == 0U) {
        (void)vl53_write_u16(sensor->address, 0x0054U, 0x8000U);
        return;
    }

    required_spads = ((uint32_t)VL53L1X_TARGET_RATE << 16) / total_rate_per_spad;
    if (required_spads > 0xFFFFU) {
        required_spads = 0xFFFFU;
    }
    (void)vl53_write_u16(sensor->address, 0x0054U, (uint16_t)required_spads);
}

static void vl53_setup_manual_calibration(vl53l1x_sensor_t *sensor)
{
    uint8_t vhv_init;
    uint8_t vhv_timeout;
    uint8_t phasecal_start;

    if (vl53_read_u8(sensor->address, 0x000BU, &vhv_init) != HAL_OK ||
        vl53_read_u8(sensor->address, 0x0008U, &vhv_timeout) != HAL_OK ||
        vl53_read_u8(sensor->address, 0x00D8U, &phasecal_start) != HAL_OK) {
        return;
    }

    if ((vl53_write_u8(sensor->address, 0x000BU, (uint8_t)(vhv_init & 0x7FU)) == HAL_OK) &&
        (vl53_write_u8(sensor->address, 0x0008U, (uint8_t)((vhv_timeout & 0x03U) + 12U)) == HAL_OK) &&
        (vl53_write_u8(sensor->address, 0x004DU, 0x01U) == HAL_OK) &&
        (vl53_write_u8(sensor->address, 0x0047U, phasecal_start) == HAL_OK)) {
        sensor->calibrated = 1U;
    }
}

static void vl53_process_ready_sample(vl53l1x_sensor_t *sensor)
{
    uint8_t results[17];
    uint8_t raw_status;
    uint8_t stream_count;
    uint16_t raw_distance;
    uint16_t filtered_distance;

    if (vl53_read(sensor->address, VL53L1X_RESULT_RANGE_STATUS_REG, results, sizeof(results)) != HAL_OK ||
        vl53_write_u8(sensor->address, VL53L1X_INTERRUPT_CLEAR_REG, 0x01U) != HAL_OK) {
        sensor_mark_error(sensor, DISTANCE_SENSOR_STATUS_I2C_ERROR, 1U);
        return;
    }

    raw_status = results[VL53L1X_REG_RESULT_STATUS];
    stream_count = results[VL53L1X_REG_RESULT_STREAM_COUNT];
    raw_distance = read_be16(&results[VL53L1X_REG_RESULT_RANGE_HI]);
    raw_distance = (uint16_t)(((uint32_t)raw_distance * 2011U + 0x0400U) / 0x0800U);

    sensor->sample.sensor_id = (uint8_t)(sensor - sensors);
    sensor->sample.timestamp_ms = HAL_GetTick();
    sensor->sample.sigma_mm = 0xFFFFU;
    sensor->sample.sequence++;
    sensor->sequence = sensor->sample.sequence;
    sensor->sample.status = 0U;
    sensor->sample.valid = 0U;

    if ((raw_status == 9U) && (stream_count != 0U)) {
        filtered_distance = median_filter(sensor, raw_distance);
        sensor->sample.distance_mm = filtered_distance;

        if ((filtered_distance >= VL53L1X_MIN_DISTANCE_MM) &&
            (filtered_distance <= VL53L1X_MAX_DISTANCE_MM)) {
            sensor->sample.valid = 1U;
            sensor->sample.status = DISTANCE_SENSOR_STATUS_VALID;
            sensor->consecutive_error_count = 0U;
            sensor->last_sample_tick = sensor->sample.timestamp_ms;
            vl53_update_dss(sensor, results);
            if (sensor->calibrated == 0U) {
                vl53_setup_manual_calibration(sensor);
            }

            if (filtered_distance <= VL53L1X_EMERGENCY_DISTANCE_MM) {
                sensor->sample.status |= DISTANCE_SENSOR_STATUS_EMERGENCY;
                emergency_sample = sensor->sample;
                emergency_pending = 1U;
            }
        } else {
            sensor->sample.distance_mm = 0xFFFFU;
            sensor->sample.status = DISTANCE_SENSOR_STATUS_OUT_OF_RANGE;
            sensor->diagnostic_pending = 1U;
            sensor->diagnostic_error_code = 2U;
        }
    } else {
        sensor->sample.distance_mm = 0xFFFFU;
        sensor->sample.status = DISTANCE_SENSOR_STATUS_LOW_QUALITY;
        sensor->diagnostic_pending = 1U;
        sensor->diagnostic_error_code = raw_status;
    }

    sensor->updated = 1U;
}

HAL_StatusTypeDef distance_sensor_init(void)
{
    HAL_StatusTypeDef overall_status = HAL_OK;
    uint8_t i;

    i2c1_configure();
    memset(sensors, 0, sizeof(sensors));
    sensors[DIST_SENSOR_FRONT].xshut_port = GPIOE;
    sensors[DIST_SENSOR_FRONT].xshut_pin = GPIO_PIN_0;
    sensors[DIST_SENSOR_FRONT].address = VL53L1X_ADDR_FRONT;
    sensors[DIST_SENSOR_LEFT].xshut_port = GPIOE;
    sensors[DIST_SENSOR_LEFT].xshut_pin = GPIO_PIN_1;
    sensors[DIST_SENSOR_LEFT].address = VL53L1X_ADDR_LEFT;
    sensors[DIST_SENSOR_RIGHT].xshut_port = GPIOE;
    sensors[DIST_SENSOR_RIGHT].xshut_pin = GPIO_PIN_2;
    sensors[DIST_SENSOR_RIGHT].address = VL53L1X_ADDR_RIGHT;

    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++) {
        sensor_set_xshut(&sensors[i], GPIO_PIN_RESET);
    }
    HAL_Delay(2U);

    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++) {
        if (vl53_init_one(&sensors[i]) != HAL_OK) {
            sensor_set_xshut(&sensors[i], GPIO_PIN_RESET);
            sensor_mark_error(&sensors[i], DISTANCE_SENSOR_STATUS_I2C_ERROR, 1U);
            overall_status = HAL_ERROR;
        }
    }

    return overall_status;
}

void distance_sensor_poll(void)
{
    vl53l1x_sensor_t *sensor;
    uint8_t gpio_status;
    uint32_t now;
    uint8_t attempts;

    if (i2c1_initialized == 0U) {
        return;
    }

    for (attempts = 0U; attempts < DISTANCE_SENSOR_COUNT; attempts++) {
        sensor = &sensors[poll_index];
        poll_index = (uint8_t)((poll_index + 1U) % DISTANCE_SENSOR_COUNT);
        if (sensor->initialized != 0U) {
            break;
        }
    }

    if ((sensor == NULL) || (sensor->initialized == 0U)) {
        return;
    }

    if (vl53_read_u8(sensor->address, VL53L1X_GPIO_STATUS_REG, &gpio_status) != HAL_OK) {
        sensor_mark_error(sensor, DISTANCE_SENSOR_STATUS_I2C_ERROR, 1U);
        return;
    }

    if ((gpio_status & 0x01U) == 0U) {
        vl53_process_ready_sample(sensor);
        return;
    }

    now = HAL_GetTick();
    if ((now - sensor->last_sample_tick) >= VL53L1X_SAMPLE_TIMEOUT_MS &&
        (sensor->sample.status & DISTANCE_SENSOR_STATUS_TIMEOUT) == 0U) {
        sensor_mark_error(sensor, DISTANCE_SENSOR_STATUS_TIMEOUT, 3U);
    }
}

uint8_t distance_sensor_take_updated(uint8_t sensor_id, distance_sensor_sample_t *sample)
{
    if ((sensor_id >= DISTANCE_SENSOR_COUNT) || (sample == NULL) || (sensors[sensor_id].updated == 0U)) {
        return 0U;
    }

    *sample = sensors[sensor_id].sample;
    sensors[sensor_id].updated = 0U;
    return 1U;
}

uint8_t distance_sensor_take_diagnostic(distance_sensor_diag_t *diagnostic)
{
    uint8_t i;

    if (diagnostic == NULL) {
        return 0U;
    }

    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++) {
        if (sensors[i].diagnostic_pending != 0U) {
            diagnostic->sensor_id = i;
            diagnostic->error_code = sensors[i].diagnostic_error_code;
            diagnostic->consecutive_error_count = sensors[i].consecutive_error_count;
            diagnostic->status = sensors[i].sample.status;
            diagnostic->last_distance_mm = sensors[i].sample.distance_mm;
            diagnostic->sequence = sensors[i].sequence;
            sensors[i].diagnostic_pending = 0U;
            return 1U;
        }
    }

    return 0U;
}

uint8_t distance_sensor_take_emergency(distance_sensor_sample_t *sample)
{
    if ((sample == NULL) || (emergency_pending == 0U)) {
        return 0U;
    }

    *sample = emergency_sample;
    emergency_pending = 0U;
    return 1U;
}
