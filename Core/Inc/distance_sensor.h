#ifndef __DISTANCE_SENSOR_H
#define __DISTANCE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* All six VL53L1X modules share I2C1: PB8=SCL, PB9=SDA.
 * Each XSHUT must be controlled separately before assigning I2C addresses. */
#define DISTANCE_SENSOR_COUNT 6U
#define DISTANCE_SENSOR_XSHUT_GPIO_Port GPIOE
#define DISTANCE_SENSOR_XSHUT_FRONT_Pin GPIO_PIN_0
#define DISTANCE_SENSOR_XSHUT_LEFT_Pin GPIO_PIN_1
#define DISTANCE_SENSOR_XSHUT_RIGHT_Pin GPIO_PIN_2
#define DISTANCE_SENSOR_XSHUT_ARM_UPPER_LEFT_Pin GPIO_PIN_5
#define DISTANCE_SENSOR_XSHUT_ARM_UPPER_RIGHT_Pin GPIO_PIN_6
#define DISTANCE_SENSOR_XSHUT_REAR_Pin GPIO_PIN_7
#define DISTANCE_SENSOR_XSHUT_PINS (DISTANCE_SENSOR_XSHUT_FRONT_Pin | \
    DISTANCE_SENSOR_XSHUT_LEFT_Pin | DISTANCE_SENSOR_XSHUT_RIGHT_Pin | \
    DISTANCE_SENSOR_XSHUT_ARM_UPPER_LEFT_Pin | DISTANCE_SENSOR_XSHUT_ARM_UPPER_RIGHT_Pin | \
    DISTANCE_SENSOR_XSHUT_REAR_Pin)

#define DISTANCE_SENSOR_STATUS_VALID        0x01U
#define DISTANCE_SENSOR_STATUS_OUT_OF_RANGE 0x02U
#define DISTANCE_SENSOR_STATUS_TIMEOUT      0x04U
#define DISTANCE_SENSOR_STATUS_I2C_ERROR    0x08U
#define DISTANCE_SENSOR_STATUS_LOW_QUALITY  0x10U
#define DISTANCE_SENSOR_STATUS_EMERGENCY    0x20U

typedef enum {
    DIST_SENSOR_FRONT = 0U,
    DIST_SENSOR_LEFT = 1U,
    DIST_SENSOR_RIGHT = 2U,
    DIST_SENSOR_ARM_UPPER_LEFT = 3U,
    DIST_SENSOR_ARM_UPPER_RIGHT = 4U,
    DIST_SENSOR_REAR = 5U
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
