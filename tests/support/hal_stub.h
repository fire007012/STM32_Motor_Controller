#ifndef HAL_STUB_H
#define HAL_STUB_H

/* Replace main.h only for native regression tests; firmware uses real HAL. */
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_CAN_STATE_READY, HAL_CAN_STATE_LISTENING } HAL_CAN_StateTypeDef;
typedef struct { unsigned index; } CAN_HandleTypeDef;
typedef struct { unsigned unused; } TIM_HandleTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define DISABLE 0U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_RX_FIFO0 0U

extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __enable_irq(void) { test_primask = 0U; }

uint32_t HAL_GetTick(void);
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *bus);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *bus);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *bus, uint32_t mailboxes);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                      const uint8_t data[8], uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *bus, uint32_t mailboxes);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *bus, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *bus, uint32_t fifo,
                                      CAN_RxHeaderTypeDef *header, uint8_t data[8]);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *bus);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer);
void Servo_HandleCanCommand(const uint8_t data[8]);
void Error_Handler(void);

#endif
