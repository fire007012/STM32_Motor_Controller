#include "can_transport.h"

#include <string.h>

typedef struct {
    CAN_TxFrame_t frame;
    can_tx_complete_cb_t callback;
    uint32_t token;
} CAN_QueuedFrame_t;

typedef struct {
    can_tx_complete_cb_t callback;
    uint32_t token;
} CAN_InFlight_t;

typedef struct {
    CAN_HandleTypeDef *bus;
    CAN_QueuedFrame_t frames[CAN_TX_QUEUE_LENGTH];
    CAN_InFlight_t in_flight[3];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint32_t failures;
} CAN_TxQueue_t;

static CAN_TxQueue_t queues[2];

static uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static CAN_TxQueue_t *find_queue(CAN_HandleTypeDef *bus)
{
    uint8_t i;
    for (i = 0U; i < 2U; i++) {
        if ((bus != NULL) && (queues[i].bus == bus)) {
            return &queues[i];
        }
    }
    return NULL;
}

void can_transport_init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2)
{
    (void)memset(queues, 0, sizeof(queues));
    queues[0].bus = can1;
    queues[1].bus = can2;
}

HAL_StatusTypeDef can_transport_send_batch_notify(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames,
                                                 uint8_t count, can_tx_complete_cb_t callback, uint32_t token)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint32_t primask;
    uint8_t i;

    if ((queue == NULL) || (frames == NULL) || (count == 0U)) {
        return HAL_ERROR;
    }
    for (i = 0U; i < count; i++) {
        if ((frames[i].header.DLC > 8U) ||
            ((frames[i].header.IDE != CAN_ID_STD) && (frames[i].header.IDE != CAN_ID_EXT)) ||
            ((frames[i].header.RTR != CAN_RTR_DATA) && (frames[i].header.RTR != CAN_RTR_REMOTE)) ||
            ((frames[i].header.IDE == CAN_ID_STD) && (frames[i].header.StdId > 0x7FFU)) ||
            ((frames[i].header.IDE == CAN_ID_EXT) && (frames[i].header.ExtId > 0x1FFFFFFFU))) {
            return HAL_ERROR;
        }
    }

    primask = enter_critical();
    if (count > (CAN_TX_QUEUE_LENGTH - queue->count)) {
        queue->failures++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    for (i = 0U; i < count; i++) {
        CAN_QueuedFrame_t *slot = &queue->frames[queue->tail];
        slot->frame = frames[i];
        slot->callback = ((i + 1U) == count) ? callback : NULL;
        slot->token = token;
        queue->tail = (uint8_t)((queue->tail + 1U) % CAN_TX_QUEUE_LENGTH);
        queue->count++;
    }
    exit_critical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef can_transport_send_batch(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames, uint8_t count)
{
    return can_transport_send_batch_notify(bus, frames, count, NULL, 0U);
}

static void complete_slot(CAN_InFlight_t *slot)
{
    can_tx_complete_cb_t callback = slot->callback;
    uint32_t token = slot->token;
    slot->callback = NULL;
    if (callback != NULL) { callback(token); }
}

void can_transport_poll_completions(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint32_t primask = enter_critical();
    if (queue != NULL) {
        for (uint8_t i = 0U; i < 3U; i++) {
            if ((queue->in_flight[i].callback != NULL) &&
                (HAL_CAN_IsTxMessagePending(bus, 1UL << i) == 0U)) {
                complete_slot(&queue->in_flight[i]);
            }
        }
    }
    exit_critical(primask);
}

HAL_StatusTypeDef can_transport_send(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header, const uint8_t data[8])
{
    CAN_TxFrame_t frame = {0};
    if ((header == NULL) || (data == NULL) || (header->DLC > 8U)) {
        return HAL_ERROR;
    }
    frame.header = *header;
    if (header->RTR == CAN_RTR_DATA) {
        (void)memcpy(frame.data, data, header->DLC);
    }
    return can_transport_send_batch(bus, &frame, 1U);
}

void can_transport_service(void)
{
    uint8_t i;
    for (i = 0U; i < 2U; i++) {
        CAN_TxQueue_t *queue = &queues[i];
        uint32_t primask = enter_critical();
        can_transport_poll_completions(queue->bus);
        /* Retain the head frame on HAL failure or mailbox exhaustion. The
         * critical section also serializes cancellation and ISR producers. */
        while ((queue->bus != NULL) && (queue->count != 0U) &&
               (HAL_CAN_GetState(queue->bus) == HAL_CAN_STATE_LISTENING) &&
               (HAL_CAN_GetTxMailboxesFreeLevel(queue->bus) != 0U)) {
            uint32_t mailbox;
            CAN_QueuedFrame_t *queued = &queue->frames[queue->head];
            CAN_TxFrame_t *frame = &queued->frame;
            if (HAL_CAN_AddTxMessage(queue->bus, &frame->header, frame->data, &mailbox) != HAL_OK) {
                queue->failures++;
                break;
            }
            for (uint8_t m = 0U; m < 3U; m++) {
                if (mailbox == (1UL << m)) {
                    /* A mailbox can finish and be reused between polls. Its
                     * old callback must fire before the new one replaces it. */
                    complete_slot(&queue->in_flight[m]);
                    queue->in_flight[m].callback = queued->callback;
                    queue->in_flight[m].token = queued->token;
                    break;
                }
            }
            queue->head = (uint8_t)((queue->head + 1U) % CAN_TX_QUEUE_LENGTH);
            queue->count--;
        }
        exit_critical(primask);
    }
}

void can_transport_cancel(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint32_t primask = enter_critical();
    if (queue != NULL) {
        queue->head = 0U;
        queue->tail = 0U;
        queue->count = 0U;
        (void)memset(queue->in_flight, 0, sizeof(queue->in_flight));
        (void)HAL_CAN_AbortTxRequest(bus, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    exit_critical(primask);
}

uint32_t can_transport_get_failure_count(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    return (queue != NULL) ? queue->failures : 0U;
}
