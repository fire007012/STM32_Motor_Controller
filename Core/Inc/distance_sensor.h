#ifndef __DISTANCE_SENSOR_H
#define __DISTANCE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DISTANCE_SENSOR_COUNT 3U

#define DISTANCE_SENSOR_STATUS_VALID        0x01U
#define DISTANCE_SENSOR_STATUS_OUT_OF_RANGE 0x02U
#define DISTANCE_SENSOR_STATUS_TIMEOUT      0x04U
#define DISTANCE_SENSOR_STATUS_I2C_ERROR    0x08U
#define DISTANCE_SENSOR_STATUS_LOW_QUALITY  0x10U
#define DISTANCE_SENSOR_STATUS_EMERGENCY    0x20U

typedef enum {
    DIST_SENSOR_FRONT = 0U,
    DIST_SENSOR_LEFT = 1U,
    DIST_SENSOR_RIGHT = 2U
} distance_sensor_id_t;

typedef struct {
    uint16_t distance_mm;
    uint16_t sigma_mm;
    uint32_t timestamp_ms;
    uint8_t sensor_id;
    uint8_t valid;
    uint8_t status;
    uint8_t sequence;
} distance_sensor_sample_t;

typedef struct {
    uint8_t sensor_id;
    uint8_t error_code;
    uint8_t consecutive_error_count;
    uint8_t status;
    uint16_t last_distance_mm;
    uint8_t sequence;
} distance_sensor_diag_t;

HAL_StatusTypeDef distance_sensor_init(void);
void distance_sensor_poll(void);
uint8_t distance_sensor_take_updated(uint8_t sensor_id, distance_sensor_sample_t *sample);
uint8_t distance_sensor_take_diagnostic(distance_sensor_diag_t *diagnostic);
uint8_t distance_sensor_take_emergency(distance_sensor_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* __DISTANCE_SENSOR_H */
