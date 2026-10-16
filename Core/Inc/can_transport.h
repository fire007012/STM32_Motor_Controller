#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "main.h"

#define CAN_TX_QUEUE_LENGTH 64U
#define CAN_ROS_TX_BACKOFF_MS 100U
#define CAN_ROS_TX_TIMEOUT_MS 20U
#define CAN_REPLY_QUEUE_LENGTH 32U
#define CAN_REPLY_LIFETIME_MS 500U
#define CAN_FORWARD_QUEUE_LENGTH 32U
#define CAN_FORWARD_LIFETIME_MS 200U
#define CAN_TX_RESERVED_SLOTS 8U

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxFrame_t;

typedef void (*can_tx_complete_cb_t)(uint32_t token);

/* Live local snapshots, also readable through SWD if CAN cannot transmit.
 * error_events counts SCE IRQ observations, not individual physical errors.
 * LEC is last_error_esr[6:4], TEC [23:16], REC [31:24]. */
typedef struct {
    uint32_t tick, mcr, btr, esr, tsr;
    uint32_t last_error_tick, last_error_esr, error_events;
    uint32_t tx_ok, tx_error, arbitration_lost, tx_timeouts;
    uint32_t reply_retries, reply_overflow, reply_expired, reply_pending;
    uint32_t rx_frames, last_rx_id;
    /* SWD-only extensions; the existing 0x103 wire format is unchanged. */
    uint32_t format_version;
    uint32_t tx_pending, tx_high_water;
    uint32_t queue_full, hal_submit_errors, hal_submit_busy;
    uint32_t latest_replaced, latest_backpressure;
    uint32_t canceled_frames, timeout_discarded;
    uint32_t error_warning_events, error_passive_events, bus_off_events;
    uint32_t lec_events[8];
    uint32_t rx_overruns;
    uint32_t tx_failures, backoff_rejected;
    /* Format 3 appends native CAN2 -> CAN1 forwarding diagnostics. */
    uint32_t forward_pending, forward_retries, forward_overflow, forward_expired;
} CAN_BusDiagnostics_t;

extern volatile CAN_BusDiagnostics_t can_bus_diagnostics[2];

void can_transport_init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
/* Safe from tasks and interrupts. HAL_OK means accepted into the software
 * queue, not acknowledged by the remote device. Batches are all-or-nothing.
 * CAN1 uses single-shot TX: ordinary FIFO errors are counted, not requeued; normal
 * arbitration loss retains the head for the next service call. During
 * its 100 ms error backoff new submissions return HAL_BUSY. CAN2 is independent. */
HAL_StatusTypeDef can_transport_send(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header, const uint8_t data[8]);
/* CAN1 periodic snapshots only. Match IDE + CAN ID + caller key (report type
 * and motor/sensor index). Replace only unsent snapshots, never an in-flight
 * frame. A batch is atomic; duplicate stream keys in a batch are invalid.
 * New streams leave 8 FIFO slots for nonreplaceable events.
 * HAL_BUSY means NOT accepted: retain/rebuild the newest data and retry; this
 * backpressure is counted separately from transport failures. Existing streams
 * can be updated even when full or during link backoff. No callbacks/coalescing
 * are allowed for commands, replies, one-shot diagnostics or multipart data. */
HAL_StatusTypeDef can_transport_send_latest(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                           const uint8_t data[8], uint16_t key);
HAL_StatusTypeDef can_transport_send_latest_batch(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames,
                                                 const uint16_t *keys, uint8_t count);
/* CAN1 application replies have independent capacity and precede telemetry.
 * Accepted even during link backoff; retry failures with the same 100 ms
 * backoff until TXOK or 500 ms from enqueue. Full returns HAL_BUSY and is
 * recorded in reply_overflow. Never retries or re-executes a motion command. */
HAL_StatusTypeDef can_transport_send_reply(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                          const uint8_t data[8]);
/* CAN2 feedback only, forwarded to CAN1 unchanged. Independent FIFO accepts
 * during backoff; priority is replies, native feedback, then telemetry.
 * Retain failed frames until TXOK or 200 ms from reception. Never coalesce
 * multipart frames or forward/retry host commands through this interface.
 * Overflow/expiry remain failures, visible locally and in 0x103. */
HAL_StatusTypeDef can_transport_forward(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[8]);
HAL_StatusTypeDef can_transport_send_batch(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames, uint8_t count);
/* Notify after the last frame leaves its hardware mailbox. Callback runs in
 * a short critical section and must not block. Cancellation removes callbacks. */
HAL_StatusTypeDef can_transport_send_batch_notify(CAN_HandleTypeDef *bus, const CAN_TxFrame_t *frames,
                                                 uint8_t count, can_tx_complete_cb_t callback, uint32_t token);
void can_transport_poll_completions(CAN_HandleTypeDef *bus);
/* Call from the motor task every millisecond; this is the only HAL TX caller. */
void can_transport_service(void);
void can_transport_cancel(CAN_HandleTypeDef *bus);
uint32_t can_transport_get_failure_count(CAN_HandleTypeDef *bus);
void can_transport_get_diagnostics(CAN_HandleTypeDef *bus, CAN_BusDiagnostics_t *out);
/* Call before HAL_CAN_IRQHandler in every enabled CAN IRQ (SCE and RX0).
 * Only captures a pending MSR.ERRI event; stale IRQs cannot overwrite it. */
void can_transport_capture_error(CAN_HandleTypeDef *bus);
void can_transport_record_rx(CAN_HandleTypeDef *bus, const CAN_RxHeaderTypeDef *header);
/* Record FIFO overrun before HAL clears RF0R.FOVR0 (RX0 ISR). */
void can_transport_capture_rx_overrun(CAN_HandleTypeDef *bus);

#endif
