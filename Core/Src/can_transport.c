#include "can_transport.h"

#include <string.h>

typedef struct {
    CAN_TxFrame_t frame;
    can_tx_complete_cb_t callback;
    uint32_t token;
    uint16_t latest_key;
    uint8_t latest;
} CAN_QueuedFrame_t;

enum { CAN_TX_NORMAL, CAN_TX_REPLY, CAN_TX_FORWARD };

typedef struct {
    can_tx_complete_cb_t callback;
    uint32_t token;
    uint32_t started;
    uint8_t active;
    uint8_t source;
} CAN_InFlight_t;

typedef struct {
    CAN_HandleTypeDef *bus;
    CAN_QueuedFrame_t frames[CAN_TX_QUEUE_LENGTH];
    CAN_InFlight_t in_flight[3];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint32_t failures;
    uint32_t last_failure_tick;
    uint8_t backoff;
} CAN_TxQueue_t;

static CAN_TxQueue_t queues[2];

static struct {
    CAN_QueuedFrame_t frames[CAN_REPLY_QUEUE_LENGTH];
    uint32_t enqueued[CAN_REPLY_QUEUE_LENGTH];
    uint8_t head, tail, count;
} replies;

static struct {
    CAN_TxFrame_t frames[CAN_FORWARD_QUEUE_LENGTH];
    uint32_t enqueued[CAN_FORWARD_QUEUE_LENGTH];
    uint8_t head, tail, count;
} forwards;

volatile CAN_BusDiagnostics_t can_bus_diagnostics[2];

static void snapshot(CAN_TxQueue_t *queue)
{
    if (queue->bus == NULL) return;
    volatile CAN_BusDiagnostics_t *diag = &can_bus_diagnostics[queue - queues];
    diag->tick = HAL_GetTick();
    diag->format_version = 3U;
    diag->mcr = queue->bus->Instance->MCR;
    diag->btr = queue->bus->Instance->BTR;
    diag->esr = queue->bus->Instance->ESR;
    diag->tsr = queue->bus->Instance->TSR;
    diag->reply_pending = queue == &queues[0] ? replies.count : 0U;
    diag->forward_pending = queue == &queues[0] ? forwards.count : 0U;
    diag->tx_pending = queue->count;
    diag->tx_failures = queue->failures;
    if (queue->count > diag->tx_high_water) diag->tx_high_water = queue->count;
}

static uint8_t backoff_active(CAN_TxQueue_t *queue)
{
    if ((queue->backoff != 0U) &&
        ((HAL_GetTick() - queue->last_failure_tick) >= CAN_ROS_TX_BACKOFF_MS)) {
        queue->backoff = 0U;
    }
    return queue->backoff;
}

static void ros_tx_failed(CAN_TxQueue_t *queue)
{
    queue->failures++;
    queue->last_failure_tick = HAL_GetTick();
    queue->backoff = 1U;
}

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
    (void)memset(&replies, 0, sizeof(replies));
    (void)memset(&forwards, 0, sizeof(forwards));
    for (uint8_t i = 0U; i < 2U; i++) {
        can_bus_diagnostics[i] = (CAN_BusDiagnostics_t){0};
    }
    queues[0].bus = can1;
    queues[1].bus = can2;
}

static uint8_t valid_header(const CAN_TxHeaderTypeDef *header)
{
    return (header != NULL) && (header->DLC <= 8U) &&
           ((header->IDE == CAN_ID_STD) || (header->IDE == CAN_ID_EXT)) &&
           ((header->RTR == CAN_RTR_DATA) || (header->RTR == CAN_RTR_REMOTE)) &&
           ((header->IDE == CAN_ID_STD) ? (header->StdId <= 0x7FFU) :
                                        (header->ExtId <= 0x1FFFFFFFU));
}

HAL_StatusTypeDef can_transport_send_reply(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                          const uint8_t data[8])
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if ((queue != &queues[0]) || !valid_header(header) || (data == NULL)) return HAL_ERROR;
    uint32_t primask = enter_critical();
    if (replies.count == CAN_REPLY_QUEUE_LENGTH) {
        queue->failures++;
        can_bus_diagnostics[0].queue_full++;
        can_bus_diagnostics[0].reply_overflow++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    CAN_QueuedFrame_t *slot = &replies.frames[replies.tail];
    *slot = (CAN_QueuedFrame_t){0};
    slot->frame.header = *header;
    if (header->RTR == CAN_RTR_DATA) (void)memcpy(slot->frame.data, data, header->DLC);
    replies.enqueued[replies.tail] = HAL_GetTick();
    replies.tail = (uint8_t)((replies.tail + 1U) % CAN_REPLY_QUEUE_LENGTH);
    replies.count++;
    can_bus_diagnostics[0].reply_pending = replies.count;
    exit_critical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef can_transport_forward(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[8])
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if ((queue != &queues[0]) || !valid_header(header) || (data == NULL)) return HAL_ERROR;
    uint32_t primask = enter_critical();
    if (forwards.count == CAN_FORWARD_QUEUE_LENGTH) {
        queue->failures++;
        can_bus_diagnostics[0].queue_full++;
        can_bus_diagnostics[0].forward_overflow++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    CAN_TxFrame_t *slot = &forwards.frames[forwards.tail];
    *slot = (CAN_TxFrame_t){.header = *header};
    if (header->RTR == CAN_RTR_DATA) (void)memcpy(slot->data, data, header->DLC);
    forwards.enqueued[forwards.tail] = HAL_GetTick();
    forwards.tail = (uint8_t)((forwards.tail + 1U) % CAN_FORWARD_QUEUE_LENGTH);
    forwards.count++;
    can_bus_diagnostics[0].forward_pending = forwards.count;
    exit_critical(primask);
    return HAL_OK;
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
        if (!valid_header(&frames[i].header)) {
            return HAL_ERROR;
        }
    }

    primask = enter_critical();
    if (backoff_active(queue)) {
        can_bus_diagnostics[queue - queues].backoff_rejected++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    if (count > (CAN_TX_QUEUE_LENGTH - queue->count)) {
        queue->failures++;
        can_bus_diagnostics[queue - queues].queue_full++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    for (i = 0U; i < count; i++) {
        CAN_QueuedFrame_t *slot = &queue->frames[queue->tail];
        slot->frame = frames[i];
        slot->callback = ((i + 1U) == count) ? callback : NULL;
        slot->token = token;
        slot->latest = 0U;
        queue->tail = (uint8_t)((queue->tail + 1U) % CAN_TX_QUEUE_LENGTH);
        queue->count++;
    }
    snapshot(queue);
    exit_critical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef can_transport_send_batch(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames, uint8_t count)
{
    return can_transport_send_batch_notify(bus, frames, count, NULL, 0U);
}

static uint8_t same_stream(const CAN_TxFrame_t *a, uint16_t a_key,
                           const CAN_TxFrame_t *b, uint16_t b_key)
{
    return a_key == b_key && a->header.IDE == b->header.IDE &&
           (a->header.IDE == CAN_ID_STD ? a->header.StdId == b->header.StdId :
                                        a->header.ExtId == b->header.ExtId);
}

HAL_StatusTypeDef can_transport_send_latest_batch(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames,
                                                 const uint16_t *keys, uint8_t count)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint8_t positions[CAN_TX_QUEUE_LENGTH];
    uint8_t needed = 0U;
    if (queue != &queues[0] || frames == NULL || keys == NULL ||
        count == 0U || count > CAN_TX_QUEUE_LENGTH) return HAL_ERROR;
    for (uint8_t i = 0U; i < count; i++) {
        if (!valid_header(&frames[i].header) || frames[i].header.RTR != CAN_RTR_DATA) return HAL_ERROR;
        for (uint8_t j = 0U; j < i; j++) {
            if (same_stream(&frames[i], keys[i], &frames[j], keys[j])) return HAL_ERROR;
        }
    }
    uint32_t primask = enter_critical();
    uint8_t head_in_flight = 0U;
    for (uint8_t m = 0U; m < 3U; m++) {
        if (queue->in_flight[m].active && queue->in_flight[m].source == CAN_TX_NORMAL) head_in_flight = 1U;
    }
    for (uint8_t i = 0U; i < count; i++) {
        positions[i] = CAN_TX_QUEUE_LENGTH;
        for (uint8_t j = head_in_flight; j < queue->count; j++) {
            uint8_t pos = (uint8_t)((queue->head + j) % CAN_TX_QUEUE_LENGTH);
            CAN_QueuedFrame_t *slot = &queue->frames[pos];
            if (slot->latest && same_stream(&frames[i], keys[i], &slot->frame, slot->latest_key)) {
                positions[i] = pos;
                break;
            }
        }
        if (positions[i] == CAN_TX_QUEUE_LENGTH) needed++;
    }
    /* Replacements need no capacity, even if nonperiodic traffic filled the
     * reserve. Preflight the entire batch before changing any snapshot. */
    if (needed && queue->count + needed > CAN_TX_QUEUE_LENGTH - CAN_TX_RESERVED_SLOTS) {
        can_bus_diagnostics[0].latest_backpressure++;
        exit_critical(primask);
        return HAL_BUSY;
    }
    for (uint8_t i = 0U; i < count; i++) {
        uint8_t pos = positions[i];
        if (pos == CAN_TX_QUEUE_LENGTH) {
            pos = queue->tail;
            queue->tail = (uint8_t)((queue->tail + 1U) % CAN_TX_QUEUE_LENGTH);
            queue->count++;
        } else {
            can_bus_diagnostics[0].latest_replaced++;
        }
        queue->frames[pos] = (CAN_QueuedFrame_t){.frame = frames[i],
                                               .latest_key = keys[i], .latest = 1U};
    }
    snapshot(queue);
    exit_critical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef can_transport_send_latest(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                           const uint8_t data[8], uint16_t key)
{
    CAN_TxFrame_t frame = {0};
    if (!valid_header(header) || data == NULL) return HAL_ERROR;
    frame.header = *header;
    (void)memcpy(frame.data, data, header->DLC);
    return can_transport_send_latest_batch(bus, &frame, &key, 1U);
}

static void complete_slot(CAN_InFlight_t *slot)
{
    can_tx_complete_cb_t callback = slot->callback;
    uint32_t token = slot->token;
    slot->callback = NULL;
    slot->active = 0U;
    if (callback != NULL) { callback(token); }
}

static void abort_requests(CAN_TxQueue_t *queue)
{
    queue->head = queue->tail = queue->count = 0U;
    (void)memset(queue->in_flight, 0, sizeof(queue->in_flight));
    (void)HAL_CAN_AbortTxRequest(queue->bus, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

void can_transport_poll_completions(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint32_t primask = enter_critical();
    if (queue != NULL) {
        snapshot(queue);
        volatile CAN_BusDiagnostics_t *diag = &can_bus_diagnostics[queue - queues];
        for (uint8_t i = 0U; i < 3U; i++) {
            CAN_InFlight_t *slot = &queue->in_flight[i];
            if (slot->active == 0U) continue;
            if (HAL_CAN_IsTxMessagePending(bus, 1UL << i) == 0U) {
                /* TX interrupts are not enabled: TSR still holds the result
                 * until HAL reuses this mailbox. Inspect before that reuse. */
                if (queue == &queues[0]) {
                    uint32_t result = bus->Instance->TSR >> (8U * i);
                    if (((result & CAN_TSR_TXOK0) == 0U) &&
                        ((result & CAN_TSR_ALST0) != 0U) &&
                        ((result & CAN_TSR_TERR0) == 0U)) {
                        /* Losing arbitration is normal traffic, not a link
                         * fault. Retain the head and try on the next service
                         * call, without firing its completion callback yet. */
                        slot->callback = NULL;
                        slot->active = 0U;
                        diag->arbitration_lost++;
                        /* A newer copy may have arrived while this snapshot
                         * was in hardware. After arbitration loss the old
                         * copy is unsent again; keep only its newer successor. */
                        CAN_QueuedFrame_t *head = &queue->frames[queue->head];
                        if (slot->source == CAN_TX_NORMAL && head->latest) {
                            for (uint8_t n = 1U; n < queue->count; n++) {
                                CAN_QueuedFrame_t *next = &queue->frames[(queue->head + n) % CAN_TX_QUEUE_LENGTH];
                                if (next->latest && same_stream(&head->frame, head->latest_key,
                                                                &next->frame, next->latest_key)) {
                                    queue->head = (uint8_t)((queue->head + 1U) % CAN_TX_QUEUE_LENGTH);
                                    queue->count--;
                                    diag->latest_replaced++;
                                    break;
                                }
                            }
                        }
                        continue;
                    }
                    if ((result & CAN_TSR_TXOK0) == 0U) {
                        diag->tx_error++;
                        slot->callback = NULL;
                        ros_tx_failed(queue);
                        if (slot->source == CAN_TX_REPLY) diag->reply_retries++;
                        if (slot->source == CAN_TX_FORWARD) diag->forward_retries++;
                    } else {
                        diag->tx_ok++;
                    }
                    if (slot->source == CAN_TX_REPLY) {
                        if ((result & CAN_TSR_TXOK0) != 0U) {
                            replies.head = (uint8_t)((replies.head + 1U) % CAN_REPLY_QUEUE_LENGTH);
                            replies.count--;
                        }
                    } else if (slot->source == CAN_TX_FORWARD) {
                        if ((result & CAN_TSR_TXOK0) != 0U) {
                            forwards.head = (uint8_t)((forwards.head + 1U) % CAN_FORWARD_QUEUE_LENGTH);
                            forwards.count--;
                        }
                    } else {
                        queue->head = (uint8_t)((queue->head + 1U) % CAN_TX_QUEUE_LENGTH);
                        queue->count--;
                    }
                } else {
                    uint32_t result = bus->Instance->TSR >> (8U * i);
                    if ((result & CAN_TSR_TXOK0) != 0U) diag->tx_ok++;
                    else {
                        diag->tx_error++;
                        queue->failures++;
                        slot->callback = NULL;
                    }
                }
                complete_slot(slot);
            } else if ((queue == &queues[0]) && (slot->source == CAN_TX_FORWARD) &&
                       ((HAL_GetTick() - forwards.enqueued[forwards.head]) >= CAN_FORWARD_LIFETIME_MS)) {
                /* A frame may enter hardware just before its reception
                 * deadline. Cancel it there too, not only when still queued. */
                (void)HAL_CAN_AbortTxRequest(bus, 1UL << i);
                slot->active = 0U;
                slot->callback = NULL;
                forwards.head = (uint8_t)((forwards.head + 1U) % CAN_FORWARD_QUEUE_LENGTH);
                forwards.count--;
                diag->forward_expired++;
                ros_tx_failed(queue);
                break;
            } else if ((queue == &queues[0]) &&
                       ((HAL_GetTick() - slot->started) >= CAN_ROS_TX_TIMEOUT_MS)) {
                /* A bus that never becomes idle (or bus-off) can leave even
                 * a single-shot request pending. Cancel stale ROS traffic. */
                diag->tx_timeouts++;
                if (slot->source == CAN_TX_REPLY) diag->reply_retries++;
                if (slot->source == CAN_TX_FORWARD) diag->forward_retries++;
                diag->timeout_discarded += queue->count;
                /* Preserve bounded replies/native feedback, discard telemetry. */
                abort_requests(queue);
                ros_tx_failed(queue);
                break;
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
        /* Keep one ROS request outstanding so a completed mailbox cannot be
         * reused by HAL before we have inspected its single-shot TX result. */
        uint8_t ros_pending = (queue == &queues[0]) &&
            (queue->in_flight[0].active || queue->in_flight[1].active ||
             queue->in_flight[2].active);
        if ((queue == &queues[0]) && (ros_pending == 0U)) {
            while ((replies.count != 0U) &&
                   ((HAL_GetTick() - replies.enqueued[replies.head]) >= CAN_REPLY_LIFETIME_MS)) {
                replies.head = (uint8_t)((replies.head + 1U) % CAN_REPLY_QUEUE_LENGTH);
                replies.count--;
                queue->failures++;
                can_bus_diagnostics[0].reply_expired++;
            }
            while ((forwards.count != 0U) &&
                   ((HAL_GetTick() - forwards.enqueued[forwards.head]) >= CAN_FORWARD_LIFETIME_MS)) {
                forwards.head = (uint8_t)((forwards.head + 1U) % CAN_FORWARD_QUEUE_LENGTH);
                forwards.count--;
                queue->failures++;
                can_bus_diagnostics[0].forward_expired++;
            }
        }
        /* Retain the head frame on HAL failure or mailbox exhaustion. The
         * critical section also serializes cancellation and ISR producers. */
        while ((queue->bus != NULL) &&
               ((queue->count != 0U) || ((queue == &queues[0]) &&
                ((replies.count != 0U) || (forwards.count != 0U)))) &&
               (ros_pending == 0U) &&
               (backoff_active(queue) == 0U) &&
               (HAL_CAN_GetState(queue->bus) == HAL_CAN_STATE_LISTENING) &&
               ((queue->bus->Instance->ESR & CAN_ESR_BOFF) == 0U) &&
               (HAL_CAN_GetTxMailboxesFreeLevel(queue->bus) != 0U)) {
            /* A notified CAN2 mailbox must have its TXOK inspected by the
             * completion poll before HAL can reuse it and clear its result.
             * This also prevents a failed TX from starting a response timer. */
            uint8_t notified_pending = 0U;
            if (queue == &queues[1]) {
                for (uint8_t m = 0U; m < 3U; m++) {
                    if (queue->in_flight[m].active && queue->in_flight[m].callback != NULL)
                        notified_pending = 1U;
                }
            }
            if (notified_pending) break;
            uint32_t mailbox;
            uint8_t reply = (queue == &queues[0]) && (replies.count != 0U);
            uint8_t forward = (queue == &queues[0]) && !reply && (forwards.count != 0U);
            CAN_QueuedFrame_t *queued = reply ? &replies.frames[replies.head] : &queue->frames[queue->head];
            CAN_TxFrame_t *frame = forward ? &forwards.frames[forwards.head] : &queued->frame;
            HAL_StatusTypeDef submitted = HAL_CAN_AddTxMessage(queue->bus, &frame->header, frame->data, &mailbox);
            if (submitted != HAL_OK) {
                volatile CAN_BusDiagnostics_t *diag = &can_bus_diagnostics[i];
                if (submitted == HAL_BUSY) diag->hal_submit_busy++;
                else diag->hal_submit_errors++;
                if (queue == &queues[0]) ros_tx_failed(queue);
                else queue->failures++;
                break;
            }
            for (uint8_t m = 0U; m < 3U; m++) {
                if (mailbox == (1UL << m)) {
                    /* A mailbox can finish and be reused between polls. Its
                     * old callback must fire before the new one replaces it. */
                    complete_slot(&queue->in_flight[m]);
                    queue->in_flight[m].callback = forward ? NULL : queued->callback;
                    queue->in_flight[m].token = forward ? 0U : queued->token;
                    queue->in_flight[m].started = HAL_GetTick();
                    queue->in_flight[m].active = 1U;
                    queue->in_flight[m].source = reply ? CAN_TX_REPLY :
                                                  forward ? CAN_TX_FORWARD : CAN_TX_NORMAL;
                    break;
                }
            }
            /* CAN1 retains its head until completion so normal arbitration
             * loss can be retried without losing ACKs or reordering batches. */
            if (queue == &queues[0]) break;
            queue->head = (uint8_t)((queue->head + 1U) % CAN_TX_QUEUE_LENGTH);
            queue->count--;
        }
        snapshot(queue);
        exit_critical(primask);
    }
}

void can_transport_cancel(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    uint32_t primask = enter_critical();
    if (queue != NULL) {
        can_bus_diagnostics[queue - queues].canceled_frames += queue->count +
            (queue == &queues[0] ? replies.count + forwards.count : 0U);
        abort_requests(queue);
        if (queue == &queues[0]) {
            replies.head = replies.tail = replies.count = 0U;
            can_bus_diagnostics[0].reply_pending = 0U;
            forwards.head = forwards.tail = forwards.count = 0U;
            can_bus_diagnostics[0].forward_pending = 0U;
        }
    }
    exit_critical(primask);
}

uint32_t can_transport_get_failure_count(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    return (queue != NULL) ? queue->failures : 0U;
}

void can_transport_get_diagnostics(CAN_HandleTypeDef *bus, CAN_BusDiagnostics_t *out)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if ((queue == NULL) || (out == NULL)) return;
    uint32_t primask = enter_critical();
    snapshot(queue);
    *out = can_bus_diagnostics[queue - queues];
    exit_critical(primask);
}

void can_transport_capture_error(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if (queue == NULL) return;
    uint32_t primask = enter_critical();
    if ((bus->Instance->MSR & CAN_MSR_ERRI) == 0U) {
        exit_critical(primask);
        return;
    }
    volatile CAN_BusDiagnostics_t *diag = &can_bus_diagnostics[queue - queues];
    snapshot(queue);
    diag->last_error_tick = diag->tick;
    diag->last_error_esr = diag->esr;
    diag->error_events++;
    if (diag->esr & CAN_ESR_EWGF) diag->error_warning_events++;
    if (diag->esr & CAN_ESR_EPVF) diag->error_passive_events++;
    if (diag->esr & CAN_ESR_BOFF) diag->bus_off_events++;
    uint32_t lec = (diag->esr >> 4) & 7U;
    if (lec != 0U && lec != 7U) diag->lec_events[lec]++;
    exit_critical(primask);
}

void can_transport_record_rx(CAN_HandleTypeDef *bus, const CAN_RxHeaderTypeDef *header)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if ((queue == NULL) || (header == NULL)) return;
    uint32_t primask = enter_critical();
    volatile CAN_BusDiagnostics_t *diag = &can_bus_diagnostics[queue - queues];
    diag->rx_frames++;
    diag->last_rx_id = header->IDE == CAN_ID_EXT ? header->ExtId : header->StdId;
    exit_critical(primask);
}

void can_transport_capture_rx_overrun(CAN_HandleTypeDef *bus)
{
    CAN_TxQueue_t *queue = find_queue(bus);
    if (queue == NULL || (bus->Instance->RF0R & CAN_RF0R_FOVR0) == 0U) return;
    uint32_t primask = enter_critical();
    can_bus_diagnostics[queue - queues].rx_overruns++;
    exit_critical(primask);
}
