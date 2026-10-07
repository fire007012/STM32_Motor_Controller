#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "main.h"

#define CAN_TX_QUEUE_LENGTH 64U

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxFrame_t;

typedef void (*can_tx_complete_cb_t)(uint32_t token);

void can_transport_init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
/* Safe from tasks and interrupts. HAL_OK means accepted into the software
 * queue, not acknowledged by the remote device. Batches are all-or-nothing. */
HAL_StatusTypeDef can_transport_send(CAN_HandleTypeDef *bus, const CAN_TxHeaderTypeDef *header, const uint8_t data[8]);
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

#endif
