#ifndef TEST_CMSIS_OS_H
#define TEST_CMSIS_OS_H

#include <stdint.h>
typedef void *osMessageQueueId_t;
typedef enum { osOK = 0, osErrorResource = -3 } osStatus_t;
osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message, uint8_t priority, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message, uint8_t *priority, uint32_t timeout);
osStatus_t osMessageQueueReset(osMessageQueueId_t queue);

#endif
