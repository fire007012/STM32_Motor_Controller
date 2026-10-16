/* Standalone hardware diagnosis. Enabled only by STM32_DIAGNOSTIC_CAN_PROBE.
 * No motor commands, sensor initialization, PWM or RTOS tasks are started. */
#include "main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* One-shot GPIO trace: TIM1 update requests DMA2 stream 5/channel 6 at 1 MHz.
 * GPIOD IDR contains both CAN1 TX (PD1) and RX (PD0). Keep the capture in
 * RAM until another request is written over SWD. This does not remux pins. */
volatile uint32_t can_probe_trace[1024];
volatile uint32_t can_probe_trace_request = 1U;
volatile uint32_t can_probe_trace_meta[4]; /* ready, sample count, DMA flags, sequence */
volatile uint32_t can_probe_idle_rx_windows[2]; /* idle windows, windows with PD0 falling edge */
volatile uint32_t can_probe_tx_rx_windows[2]; /* own transmissions, transmissions with RX edge */

static uint32_t probe_trace_begin(uint32_t sequence)
{
    if (can_probe_trace_request == 0U) return 0U;
    can_probe_trace_request = 0U;
    can_probe_trace_meta[0] = 0U;
    can_probe_trace_meta[3] = sequence;
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM1->CR1 = 0U;
    TIM1->DIER = 0U;
    TIM1->PSC = 0U;
    TIM1->ARR = 71U;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    DMA2_Stream5->CR = 0U;
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA2_Stream5->PAR = (uint32_t)&GPIOD->IDR;
    DMA2_Stream5->M0AR = (uint32_t)can_probe_trace;
    DMA2_Stream5->NDTR = 1024U;
    DMA2_Stream5->FCR = 0U;
    DMA2_Stream5->CR = (6UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                       DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_PL_1 | DMA_SxCR_EN;
    TIM1->DIER = TIM_DIER_UDE;
    TIM1->CR1 = TIM_CR1_CEN;
    return 1U;
}

static void probe_trace_end(void)
{
    uint32_t start = HAL_GetTick();
    while (DMA2_Stream5->NDTR != 0U && (HAL_GetTick() - start) < 5U) { }
    TIM1->CR1 = 0U;
    TIM1->DIER = 0U;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    can_probe_trace_meta[1] = 1024U - DMA2_Stream5->NDTR;
    can_probe_trace_meta[2] = DMA2->HISR;
    can_probe_trace_meta[0] = DMA2_Stream5->NDTR == 0U ? 1U : 2U;
}

/* Inspect this symbol with SWD while the CPU is running. */
volatile struct {
    uint32_t magic;
    uint32_t phase;             /* 1=loopback, 2=normal probe, 0xFF=fatal */
    uint32_t failure;           /* 1=init, 2=send, 3=timeout, 4=TX, 5=RX, 6=compare */
    uint32_t loopback_passed;
    uint32_t attempts;
    uint32_t completed;
    uint32_t tx_ok;
    uint32_t tx_error;
    uint32_t arbitration_lost;
    uint32_t last_esr;
    uint32_t last_tsr;
    uint32_t btr;
    uint32_t rcc_cr;
    uint32_t rcc_pllcfgr;
    uint32_t rcc_cfgr;
    uint32_t tick;
    uint32_t rx_count;
    uint32_t last_rx_id;
    uint32_t last_rx_dlc;
    uint32_t last_rx_data_low;
} can_probe_result;

static void probe_fatal(uint32_t reason)
{
    can_probe_result.failure = reason;
    can_probe_result.phase = 0xFFU;
    for (;;) { can_probe_result.tick = HAL_GetTick(); }
}

static void probe_start(uint32_t mode)
{
    CAN_FilterTypeDef filter = {0};
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6U;
    hcan1.Init.Mode = mode;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) probe_fatal(1U);
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK) probe_fatal(1U);
    can_probe_result.btr = CAN1->BTR;
}

static uint32_t probe_wait(uint32_t mailbox)
{
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U) {
        if ((HAL_GetTick() - start) >= 20U) {
            (void)HAL_CAN_AbortTxRequest(&hcan1, mailbox);
            return 0U;
        }
    }
    return 1U;
}

void can_probe_run(void)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8] = {0};
    uint8_t received[8];
    uint32_t mailbox;
    uint32_t sequence = 0U;

    can_probe_result.magic = 0x43414E44U; /* CAND */
    can_probe_result.rcc_cr = RCC->CR;
    can_probe_result.rcc_pllcfgr = RCC->PLLCFGR;
    can_probe_result.rcc_cfgr = RCC->CFGR;
#if defined(STM32_DIAGNOSTIC_HSE_DIRECT) && STM32_DIAGNOSTIC_HSE_DIRECT
    can_probe_result.phase = 3U;
    for (;;) { can_probe_result.tick = HAL_GetTick(); }
#endif
    can_probe_result.phase = 1U;
    probe_start(CAN_MODE_SILENT_LOOPBACK);
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    /* Exercise actual HAL mailbox packing and hardware CRC/stuffing with
     * the observed frame, alternating patterns, different IDs and all DLCs. */
    for (uint32_t n = 0U; n < 32U; n++) {
        header.StdId = n == 0U ? 0x101U : 0x120U + n;
        header.DLC = n == 0U ? 8U : n % 9U;
        for (uint32_t i = 0U; i < 8U; i++)
            data[i] = n == 0U ? (i == 0U ? 1U : 0U) :
                      (uint8_t)((n & 1U ? 0xAAU : 0x55U) ^ (i * n));
        if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)
            probe_fatal(2U);
        if (!probe_wait(mailbox)) probe_fatal(3U);
        uint32_t shift = mailbox == CAN_TX_MAILBOX0 ? 0U :
                         mailbox == CAN_TX_MAILBOX1 ? 8U : 16U;
        if ((CAN1->TSR & (CAN_TSR_TXOK0 << shift)) == 0U) probe_fatal(4U);
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U ||
            HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, received) != HAL_OK)
            probe_fatal(5U);
        if (rx_header.StdId != header.StdId || rx_header.IDE != header.IDE ||
            rx_header.RTR != header.RTR || rx_header.DLC != header.DLC ||
            memcmp(data, received, header.DLC) != 0) probe_fatal(6U);
        can_probe_result.loopback_passed++;
    }
    if (HAL_CAN_Stop(&hcan1) != HAL_OK || HAL_CAN_DeInit(&hcan1) != HAL_OK)
        probe_fatal(1U);
    probe_start(CAN_MODE_NORMAL);
    /* Observe RX edges during the one-second idle periods. The EXTI line must
     * be unmasked for its pending latch to capture edges on this target. Keep
     * the NVIC IRQ disabled; AF input remains CAN1_RX and CAN timing is unchanged. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) |
                          SYSCFG_EXTICR1_EXTI0_PD; /* EXTI0 source = PD0 */
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    EXTI->IMR |= GPIO_PIN_0;
    EXTI->FTSR |= GPIO_PIN_0;
    can_probe_result.phase = 2U;
    header.StdId = 0x120U;
    header.DLC = 8U;
    for (;;) {
        data[0] = 0xCAU;
        data[1] = 0x4EU;
        data[2] = (uint8_t)sequence;
        data[3] = (uint8_t)(sequence >> 8);
        data[4] = 0x55U;
        data[5] = 0xAAU;
        data[6] = 0x00U;
        data[7] = 0xFFU;
        uint32_t tracing = probe_trace_begin(sequence);
        EXTI->PR = GPIO_PIN_0;
        can_probe_result.attempts++;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK) {
            can_probe_result.failure = 2U;
        } else if (!probe_wait(mailbox)) {
            can_probe_result.failure = 3U;
        } else {
            uint32_t shift = mailbox == CAN_TX_MAILBOX0 ? 0U :
                             mailbox == CAN_TX_MAILBOX1 ? 8U : 16U;
            uint32_t tsr = CAN1->TSR;
            can_probe_result.completed++;
            if ((tsr & (CAN_TSR_TXOK0 << shift)) != 0U) can_probe_result.tx_ok++;
            if ((tsr & (CAN_TSR_TERR0 << shift)) != 0U) can_probe_result.tx_error++;
            if ((tsr & (CAN_TSR_ALST0 << shift)) != 0U) can_probe_result.arbitration_lost++;
        }
        can_probe_tx_rx_windows[0]++;
        if ((EXTI->PR & GPIO_PIN_0) != 0U) can_probe_tx_rx_windows[1]++;
        if (tracing != 0U) probe_trace_end();
        can_probe_result.last_esr = CAN1->ESR;
        can_probe_result.last_tsr = CAN1->TSR;
        can_probe_result.tick = HAL_GetTick();
        /* Drain host traffic without invoking the application parser. */
        while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0U) {
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, received) == HAL_OK) {
                can_probe_result.rx_count++;
                can_probe_result.last_rx_id = rx_header.IDE == CAN_ID_STD ?
                                              rx_header.StdId : rx_header.ExtId;
                can_probe_result.last_rx_dlc = rx_header.DLC;
                can_probe_result.last_rx_data_low =
                    (uint32_t)received[0] | ((uint32_t)received[1] << 8) |
                    ((uint32_t)received[2] << 16) | ((uint32_t)received[3] << 24);
            }
        }
        sequence++;
        HAL_Delay(1U); /* Exclude this transmission's error/delimiter bits. */
        EXTI->PR = GPIO_PIN_0;
        HAL_Delay(1000U);
        if ((EXTI->PR & GPIO_PIN_0) != 0U) can_probe_idle_rx_windows[1]++;
        can_probe_idle_rx_windows[0]++;
    }
}
