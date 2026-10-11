#include "arm_servo.h"

#include <string.h>

typedef struct {
    uint16_t travel_deg;
    uint16_t min_us;
    uint16_t max_us;
    uint16_t zero_us;
    int8_t direction;
    int16_t min_angle;
    int16_t max_angle;
} arm_servo_calibration_t;

typedef struct {
    uint8_t stable;
    uint8_t armed;
    uint8_t pending;
    uint32_t since_ms;
} arm_key_t;

static TIM_HandleTypeDef arm_timer;
static volatile uint8_t arm_ready;
static volatile int16_t target_deg[ARM_SERVO_ACTIVE_COUNT];
static arm_key_t keys[2];
static uint8_t chord_blocked;
static const uint32_t pwm_channels[ARM_SERVO_CHANNEL_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};
static const arm_servo_calibration_t calibration[ARM_SERVO_ACTIVE_COUNT] = {
    {ARM_SERVO_1_TRAVEL_DEG, ARM_SERVO_1_MIN_PULSE_US,
     ARM_SERVO_1_MAX_PULSE_US, ARM_SERVO_1_ZERO_PULSE_US,
     ARM_SERVO_1_DIRECTION, ARM_SERVO_1_MIN_ANGLE_DEG,
     ARM_SERVO_1_MAX_ANGLE_DEG},
    {ARM_SERVO_2_TRAVEL_DEG, ARM_SERVO_2_MIN_PULSE_US,
     ARM_SERVO_2_MAX_PULSE_US, ARM_SERVO_2_ZERO_PULSE_US,
     ARM_SERVO_2_DIRECTION, ARM_SERVO_2_MIN_ANGLE_DEG,
     ARM_SERVO_2_MAX_ANGLE_DEG}
};

static int32_t angle_to_us(const arm_servo_calibration_t *cfg, int16_t angle)
{
    int32_t numerator = (int32_t)angle * cfg->direction *
                        ((int32_t)cfg->max_us - cfg->min_us);
    int32_t half = cfg->travel_deg / 2;
    int32_t delta = (numerator >= 0) ?
                    (numerator + half) / cfg->travel_deg :
                    -((-numerator + half) / cfg->travel_deg);
    return (int32_t)cfg->zero_us + delta;
}

static uint8_t calibration_valid(const arm_servo_calibration_t *cfg)
{
    if ((cfg->travel_deg == 0U) || (cfg->min_us < 400U) ||
        (cfg->max_us > 2600U) || (cfg->min_us >= cfg->max_us) ||
        (cfg->zero_us < cfg->min_us) || (cfg->zero_us > cfg->max_us) ||
        ((cfg->direction != 1) && (cfg->direction != -1)) ||
        (cfg->min_angle > 0) || (cfg->max_angle < 0) ||
        (cfg->min_angle >= cfg->max_angle) ||
        (cfg->min_angle < -360) || (cfg->max_angle > 360)) {
        return 0U;
    }
    int32_t first = angle_to_us(cfg, cfg->min_angle);
    int32_t last = angle_to_us(cfg, cfg->max_angle);
    return ((first >= cfg->min_us) && (first <= cfg->max_us) &&
            (last >= cfg->min_us) && (last <= cfg->max_us)) ? 1U : 0U;
}

static void step_both(int16_t step)
{
    uint8_t i;
    for (i = 0U; i < ARM_SERVO_ACTIVE_COUNT; ++i) {
        int32_t next = (int32_t)target_deg[i] + step;
        if (next < calibration[i].min_angle) next = calibration[i].min_angle;
        if (next > calibration[i].max_angle) next = calibration[i].max_angle;
        target_deg[i] = (int16_t)next;
        __HAL_TIM_SET_COMPARE(&arm_timer, pwm_channels[i],
                             (uint32_t)angle_to_us(&calibration[i], (int16_t)next));
    }
}

HAL_StatusTypeDef ArmServo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint8_t i;
    uint32_t clock_hz = HAL_RCC_GetPCLK2Freq();

    arm_ready = 0U;
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) clock_hz *= 2U;
    if ((clock_hz < 1000000U) || ((clock_hz % 1000000U) != 0U)) return HAL_ERROR;
    for (i = 0U; i < ARM_SERVO_ACTIVE_COUNT; ++i) {
        if (!calibration_valid(&calibration[i])) return HAL_ERROR;
    }

    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* Four independent outputs on one dedicated timer. SDIO must not use
     * PC8/PC9 while they are assigned to TIM8. No interrupts/DMA required. */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    memset(&arm_timer, 0, sizeof(arm_timer));
    arm_timer.Instance = TIM8;
    arm_timer.Init.Prescaler = clock_hz / 1000000U - 1U;
    arm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    arm_timer.Init.Period = ARM_SERVO_FRAME_US - 1U;
    arm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    arm_timer.Init.RepetitionCounter = 0U;
    arm_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&arm_timer) != HAL_OK) return HAL_ERROR;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    for (i = 0U; i < ARM_SERVO_CHANNEL_COUNT; ++i) {
        if (HAL_TIM_PWM_ConfigChannel(&arm_timer, &oc, pwm_channels[i]) != HAL_OK) return HAL_ERROR;
    }
    for (i = 0U; i < ARM_SERVO_ACTIVE_COUNT; ++i) {
        target_deg[i] = 0;
        __HAL_TIM_SET_COMPARE(&arm_timer, pwm_channels[i], calibration[i].zero_us);
    }
    /* Latch the preloaded zero pulses before enabling the first frame. */
    if (HAL_TIM_GenerateEvent(&arm_timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK) return HAL_ERROR;
    for (i = 0U; i < ARM_SERVO_ACTIVE_COUNT; ++i) {
        if (HAL_TIM_PWM_Start(&arm_timer, pwm_channels[i]) != HAL_OK) {
            /* A failed second output must not leave the first commanded. */
            uint8_t j;
            for (j = 0U; j < i; ++j) (void)HAL_TIM_PWM_Stop(&arm_timer, pwm_channels[j]);
            return HAL_ERROR;
        }
    }

    /* Ignore keys held during reset until a debounced release. */
    memset(keys, 0, sizeof(keys));
    keys[0].stable = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET);
    keys[1].stable = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET);
    keys[0].armed = !keys[0].stable;
    keys[1].armed = !keys[1].stable;
    keys[0].since_ms = keys[1].since_ms = HAL_GetTick();
    chord_blocked = keys[0].stable && keys[1].stable;
    arm_ready = 1U;
    return HAL_OK;
}

static uint8_t key_pressed(arm_key_t *key, uint8_t pressed, uint32_t now)
{
    if (pressed == key->stable) {
        key->pending = 0U;
        return 0U;
    }
    if (!key->pending) {
        key->pending = 1U;
        key->since_ms = now;
        return 0U;
    }
    if ((uint32_t)(now - key->since_ms) < 20U) return 0U;
    key->stable = pressed;
    key->pending = 0U;
    if (!pressed) {
        key->armed = 1U;
        return 0U;
    }
    if (!key->armed) return 0U;
    key->armed = 0U;
    return 1U;
}

void ArmServo_PollButtons(uint8_t k0_pressed, uint8_t k1_pressed)
{
    uint8_t first, second;
    uint32_t now;
    if (!arm_ready) return;
    k0_pressed = !!k0_pressed;
    k1_pressed = !!k1_pressed;
    now = HAL_GetTick();
    first = key_pressed(&keys[0], k0_pressed, now);
    second = key_pressed(&keys[1], k1_pressed, now);
    /* Conflicting directions block motion until BOTH keys are released. */
    if (k0_pressed && k1_pressed) chord_blocked = 1U;
    if (chord_blocked) {
        if (!k0_pressed && !k1_pressed && !keys[0].stable && !keys[1].stable &&
            !keys[0].pending && !keys[1].pending) chord_blocked = 0U;
        return;
    }
    if (first && !k1_pressed) step_both(ARM_SERVO_STEP_DEG);
    else if (second && !k0_pressed) step_both(-ARM_SERVO_STEP_DEG);
}

int16_t ArmServo_GetTargetAngle(uint8_t channel)
{
    if ((channel < 1U) || (channel > ARM_SERVO_ACTIVE_COUNT)) return 0;
    return target_deg[channel - 1U];
}
