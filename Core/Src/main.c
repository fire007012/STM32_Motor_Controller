/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_protocol.h"
#include "can_transport.h"
#include "zdt_can_driver.h"
#include "motor_control.h"
#include "zdt_status.h"
#include "distance_sensor.h"
#include "chassis_collision.h"
#include "arm_servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATUS_TASK_PERIOD_MS              20U
#define STATUS_QUERY_BUDGET_PER_CYCLE      1U
#define ROS_HEARTBEAT_TIMEOUT_FLAG         0x80U
#define BUTTON_DEBOUNCE_SAMPLES            3U
#define SERVO_PWM_MIN_TICKS                55U
#define SERVO_PWM_MAX_TICKS                250U
#define SERVO_CAN_CMD_ID                   0x700U
#define SERVO_CAN_CMD_CODE                 0x20U
#define SERVO_BOARD_STANDARD                1U
#define SERVO_BOARD_CAM                     2U
#define SERVO_ACTIVE_BOARD                  SERVO_BOARD_STANDARD
#define SERVO_STARTUP_ANGLE                 180


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for MotorControlTask */
osThreadId_t motorControlTaskHandle;
const osThreadAttr_t motorControlTask_attributes = {
  .name = "MotorCtrlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for StatusTask */
osThreadId_t statusTaskHandle;
const osThreadAttr_t statusTask_attributes = {
  .name = "StatusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for HeartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};

/* Definitions for SensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartStatusTask(void *argument);
void StartHeartbeatTask(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static uint8_t CAN_TryStart(CAN_HandleTypeDef *hcan);
static void CAN_ServiceStartup(void);
static HAL_StatusTypeDef CAN1_TrySend(const CAN_TxHeaderTypeDef *txHeader, uint8_t txData[8]);
static void CAN1_SendStatus(uint8_t motor_idx);
static HAL_StatusTypeDef CAN1_SendAck(uint16_t seq, uint8_t cmd, uint8_t result);
static void CAN1_SendStats(void);
static void CAN1_SendEmergencyEvent(uint8_t motor_idx, uint8_t reason_code, int32_t speed_rpm);
static void CAN1_SendButtonEvent(uint8_t event_code, uint8_t result, uint16_t seq);
static HAL_StatusTypeDef CAN1_SendDistance(const distance_sensor_sample_t *sample);
static HAL_StatusTypeDef CAN1_SendSensorDiag(const distance_sensor_diag_t *diagnostic);
static HAL_StatusTypeDef Servo_Init(void);
static void Servo_SetBoth(int16_t angle_deg);
static void Servo_PollButtons(void);
static void Servo_Service(void);
static void Buttons_Service(void);
static void WkupDemo_Service(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint8_t servo_return_pending = 0U;
static volatile uint8_t servo_return_angle = 0U;
static volatile uint8_t servo_return_mask = 0U;
static volatile uint32_t servo_return_tick = 0U;
static volatile uint8_t servo_remote_owned = 0U;
static volatile uint8_t servo_initialized = 0U;
typedef enum {
  WKUP_DEMO_IDLE = 0,
  WKUP_DEMO_ARM_FORWARD,
  WKUP_DEMO_SERVO_OPEN,
  WKUP_DEMO_SERVO_CLOSE,
  WKUP_DEMO_ARM_RETURN
} WkupDemoState_t;
static WkupDemoState_t wkup_demo_state = WKUP_DEMO_IDLE;
static uint32_t wkup_demo_stage_tick = 0U;
static uint8_t can1_started = 0U;
static uint8_t can2_started = 0U;
static uint8_t can_motors_enabled = 0U;

static void CAN1_SendButtonEvent(uint8_t event_code, uint8_t result, uint16_t seq)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};

  txData[0] = event_code;
  txData[1] = ROS_BUTTON_EVENT_PRESSED;
  txData[2] = ROS_BUTTON_SOURCE_PHYSICAL;
  txData[3] = result;
  txData[4] = (uint8_t)(seq >> 8);
  txData[5] = (uint8_t)(seq & 0xFFU);

  txHeader.StdId = ROS_CAN_BUTTON_EVENT_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  (void)can_transport_send_reply(&hcan1, &txHeader, txData);
}

static void Buttons_Service(void)
{
  static uint8_t initialized = 0U;
  static uint8_t start_stable = 0U;
  static uint8_t start_debounce = 0U;
  uint8_t start_now;

  start_now = (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  /* Ignore a start key held through reset. */
  if (initialized == 0U)
  {
    start_stable = start_now;
    initialized = 1U;
  }

  if (start_now != start_stable)
  {
    if (++start_debounce >= BUTTON_DEBOUNCE_SAMPLES)
    {
      start_stable = start_now;
      start_debounce = 0U;
      if (start_stable != 0U)
      {
        uint16_t seq = 0U;
        uint8_t accepted = 0U;

        /* ROS owns the heartbeat and motion cache.  The local key only adds
         * an ordered sync marker after commands already accepted by firmware. */
        if (motor_is_ros_ready(CHASSIS_HEARTBEAT_TIMEOUT_MS) != 0U)
        {
          seq = motor_allocate_cmd_seq();
          accepted = motor_request_local_sync_start(seq);
        }
        CAN1_SendButtonEvent(ROS_BUTTON_EVENT_START,
                             (accepted != 0U) ? ROS_BUTTON_RESULT_ACCEPTED : ROS_BUTTON_RESULT_REJECTED,
                             (accepted != 0U) ? seq : 0U);
      }
    }
  }
  else
  {
    start_debounce = 0U;
  }
}

static uint32_t Servo_AngleToPulse(int16_t angle_deg)
{
  int32_t angle = angle_deg;
  int32_t span = (int32_t)SERVO_PWM_MAX_TICKS - (int32_t)SERVO_PWM_MIN_TICKS;

  /* The old servo is mechanically limited to 0..180 degrees.  Keep -10
   * as a calibration position instead of allowing signed-to-unsigned wrap. */
  if (angle < -10) angle = -10;
  if (angle > 180) angle = 180;

  return (uint32_t)((int32_t)SERVO_PWM_MIN_TICKS +
                    (span * angle + ((angle >= 0) ? 90 : -90)) / 180);
}

static void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle_deg)
{
  __HAL_TIM_SET_COMPARE(htim, channel, Servo_AngleToPulse(angle_deg));
}

static void Servo_SetBoth(int16_t angle_deg)
{
  Servo_SetAngle(&htim3, TIM_CHANNEL_3, angle_deg);
  Servo_SetAngle(&htim4, TIM_CHANNEL_3, angle_deg);
}

static HAL_StatusTypeDef Servo_Init(void)
{
  /* Both drug-box servos are positional SG90s. RESET commands the same
   * calibrated 180-degree endpoint on PB0 and PD14 and keeps PWM active. */
  Servo_SetBoth(SERVO_STARTUP_ANGLE);

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  servo_initialized = 1U;
  return HAL_OK;
}

static uint8_t Servo_ChecksumOk(const uint8_t data[8])
{
  uint8_t sum = 0U;
  uint8_t i;

  for (i = 0U; i < 7U; i++)
  {
    sum = (uint8_t)(sum + data[i]);
  }

  return (sum == data[7]) ? 1U : 0U;
}

uint8_t Servo_HandlePositionCanCommand(const uint8_t data[8])
{
  if (wkup_demo_state != WKUP_DEMO_IDLE) return 3U;
  if ((data == NULL) || (data[0] != POSITION_SERVO_CMD) || !Servo_ChecksumOk(data) ||
      (data[1] == 0U) || ((data[1] & ~0x03U) != 0U) || data[4] || data[5]) return 1U;
  if (!servo_initialized) return 3U;
  if (((data[1] & 1U) && data[2] > 180U) ||
      ((data[1] & 2U) && data[3] > 180U)) return 2U;
  servo_return_pending = 0U;
  servo_remote_owned = 1U;
  if (data[1] & 1U) Servo_SetAngle(&htim3, TIM_CHANNEL_3, data[2]);
  if (data[1] & 2U) Servo_SetAngle(&htim4, TIM_CHANNEL_3, data[3]);
  return 0U;
}

void Servo_HandleCanCommand(const uint8_t data[8])
{
  uint16_t hold_ms;
  uint32_t now;

  if (wkup_demo_state != WKUP_DEMO_IDLE || servo_remote_owned || (data == NULL) ||
      (data[0] != SERVO_CAN_CMD_CODE) || (Servo_ChecksumOk(data) == 0U))
  {
    return;
  }

  servo_return_pending = 0U;

  if ((data[1] & 0x01U) != 0U)
  {
    Servo_SetAngle(&htim3, TIM_CHANNEL_3, data[2]);
  }

  if ((data[1] & 0x02U) != 0U)
  {
    Servo_SetAngle(&htim4, TIM_CHANNEL_3, data[3]);
  }

  hold_ms = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
  if (hold_ms != 0U)
  {
    servo_return_angle = data[4];
    servo_return_mask = (uint8_t)(data[1] & 0x03U);
    now = HAL_GetTick();
    servo_return_tick = now + hold_ms;
    servo_return_pending = 1U;
  }
}

void Servo_AcquireRemoteControl(void)
{
  servo_return_pending = 0U;
  servo_remote_owned = 1U;
}

void Servo_ReleaseLocalControl(void)
{
  servo_remote_owned = 0U;
}

/* K0/K1 and WK_UP are task-polled, not scanned from TIM6. The sequence only
 * changes PWM targets and advances from ticks; it never waits or blocks. */

#if (ARM_SERVO_KEY_TEST_ENABLE == 0U)
static void Servo_SetLocalBoth(int16_t angle)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  /* ARM_SERVO_KEY_TEST_ENABLE=0 explicitly assigns K0/K1 to local SG90
   * bench control, so a stale remote arm owner must not disable this test. */
  servo_return_pending = 0U;
  Servo_SetBoth(angle);
  if (!primask) __enable_irq();
}
#endif

static void Servo_PollButtons(void)
{
  if (wkup_demo_state != WKUP_DEMO_IDLE) return;
#if (ARM_SERVO_KEY_TEST_ENABLE != 0U)
  ArmServo_PollButtons(
      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET,
      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET);
#else
  static uint8_t k0_stable = 0U, k1_stable = 0U;
  static uint8_t k0_debounce = 0U, k1_debounce = 0U;
  uint8_t k0_now = 0U, k1_now = 0U;

#if (SERVO_ACTIVE_BOARD == SERVO_BOARD_STANDARD)
  k0_now = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET) ? 1U : 0U;
  k1_now = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1U : 0U;
#else
  static uint8_t wkup_stable = 0U;
  static uint8_t wkup_debounce = 0U;
  uint8_t wkup_now = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
  k0_now = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) ? 1U : 0U;
#endif

#define SERVO_DEBOUNCE(now, stable, counter, action) do { \
  if ((now) != (stable)) { \
    if (++(counter) >= 20U) { \
      (stable) = (now); (counter) = 0U; \
      if ((stable) != 0U) { action; } \
    } \
  } else { (counter) = 0U; } \
} while (0)
#if (SERVO_ACTIVE_BOARD == SERVO_BOARD_STANDARD)
  SERVO_DEBOUNCE(k0_now, k0_stable, k0_debounce, Servo_SetLocalBoth(0));
  SERVO_DEBOUNCE(k1_now, k1_stable, k1_debounce, Servo_SetLocalBoth(180));
  /* WK_UP is not used by this positional-servo test. */
#else
  SERVO_DEBOUNCE(k0_now, k0_stable, k0_debounce, Servo_SetBoth(180));
  SERVO_DEBOUNCE(wkup_now, wkup_stable, wkup_debounce, Servo_SetBoth(0));
#endif
#undef SERVO_DEBOUNCE
#endif
}

static void Servo_Service(void)
{
  if ((servo_return_pending != 0U) &&
      ((int32_t)(HAL_GetTick() - servo_return_tick) >= 0))
  {
    servo_return_pending = 0U;
    if ((servo_return_mask & 0x01U) != 0U)
    {
      Servo_SetAngle(&htim3, TIM_CHANNEL_3, servo_return_angle);
    }
    if ((servo_return_mask & 0x02U) != 0U)
    {
      Servo_SetAngle(&htim4, TIM_CHANNEL_3, servo_return_angle);
    }
  }
}

static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef filter = {0};

  /* CAN1: accept all frames in FIFO0 with filter bank 0. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  /* CAN2: receive Y42 extended-frame responses in FIFO0 with filter bank 14. */
  filter.FilterBank = 14;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t CAN_TryStart(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_StateTypeDef state;

  if (hcan == NULL)
  {
    return 0U;
  }

  state = HAL_CAN_GetState(hcan);
  if (state == HAL_CAN_STATE_LISTENING)
  {
    return 1U;
  }

  if ((state != HAL_CAN_STATE_READY) && (HAL_CAN_Init(hcan) != HAL_OK))
  {
    return 0U;
  }

  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
      CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK)
  {
    (void)HAL_CAN_Stop(hcan);
    return 0U;
  }

  return 1U;
}

static void CAN_ServiceStartup(void)
{
  if (can1_started == 0U)
  {
    can1_started = CAN_TryStart(&hcan1);
  }

  if (can2_started == 0U)
  {
    can2_started = CAN_TryStart(&hcan2);
  }
}

static HAL_StatusTypeDef CAN1_TrySend(const CAN_TxHeaderTypeDef *txHeader, uint8_t txData[8])
{
  if (txHeader->IDE == CAN_ID_STD &&
      (txHeader->StdId == ROS_CAN_DISTANCE_ID || txHeader->StdId == ROS_CAN_STATS_ID))
  {
    uint16_t key = txHeader->StdId == ROS_CAN_DISTANCE_ID ?
        (uint16_t)(((uint16_t)txData[0] << 8) | txData[1]) : txData[0];
    return can_transport_send_latest(&hcan1, txHeader, txData, key);
  }
  return can_transport_send(&hcan1, txHeader, txData);
}

static void CAN1_SendStatus(uint8_t motor_idx)
{
  CAN_TxFrame_t frames[5] = {0};
  uint16_t keys[5];
  Motor_State_t state;
  uint8_t count = 0U;
  uint8_t report_mask;
  uint32_t primask;

  if (motor_idx >= MOTOR_COUNT) return;
  report_mask = motor_get_report_mask();
  primask = __get_PRIMASK();
  __disable_irq();
  state = motors[motor_idx];
  if (primask == 0U) __enable_irq();

  for (uint8_t type = 1U; type <= 5U; type++)
  {
    uint8_t mask = type <= 3U ? (uint8_t)(1U << (type - 1U)) : MOTOR_REPORT_TARGET;
    CAN_TxFrame_t *frame;
    int32_t value;
    if ((report_mask & mask) == 0U) continue;
    keys[count] = (uint16_t)(((uint16_t)type << 8) | motor_idx);
    frame = &frames[count++];
    frame->header.StdId = ROS_CAN_STATUS_ID;
    frame->header.IDE = CAN_ID_STD;
    frame->header.RTR = CAN_RTR_DATA;
    frame->header.DLC = 8U;
    frame->data[0] = type;
    frame->data[1] = motor_idx;
    if (type == 1U)
    {
      frame->data[2] = (uint8_t)state.status_word;
      frame->data[3] = (uint8_t)(state.status_word >> 8);
      frame->data[4] = (uint8_t)state.fault_code;
      frame->data[5] = (uint8_t)(state.fault_code >> 8);
      frame->data[6] = state.fault_flag;
      continue;
    }
    value = type == 2U ? state.position_feedback :
            type == 3U ? state.current_velocity :
            type == 4U ? state.target_velocity : state.target_position;
    frame->data[2] = (uint8_t)value;
    frame->data[3] = (uint8_t)((uint32_t)value >> 8);
    frame->data[4] = (uint8_t)((uint32_t)value >> 16);
    frame->data[5] = (uint8_t)((uint32_t)value >> 24);
  }
  /* Atomically replace each motor's snapshots. HAL_BUSY accepts none; the next
   * status cycle rebuilds all selected types from current state, never backlog. */
  if (count != 0U) (void)can_transport_send_latest_batch(&hcan1, frames, keys, count);
}

static HAL_StatusTypeDef CAN1_SendDistance(const distance_sensor_sample_t *sample)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8] = {0};

    if (sample == NULL)
    {
      return HAL_ERROR;
    }

    txData[0] = 0x01U;
    txData[1] = sample->sensor_id;
    txData[2] = (uint8_t)(sample->distance_mm & 0xFFU);
    txData[3] = (uint8_t)(sample->distance_mm >> 8);
    txData[4] = (uint8_t)(sample->sigma_mm & 0xFFU);
    txData[5] = (uint8_t)(sample->sigma_mm >> 8);
    txData[6] = sample->status;
    txData[7] = sample->sequence;

    txHeader.StdId = ROS_CAN_DISTANCE_ID;
    txHeader.ExtId = 0U;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8U;
    txHeader.TransmitGlobalTime = DISABLE;
    return CAN1_TrySend(&txHeader, txData);
  }

static HAL_StatusTypeDef CAN1_SendSensorDiag(const distance_sensor_diag_t *diagnostic)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8] = {0};

    if (diagnostic == NULL)
    {
      return HAL_ERROR;
    }

    txData[0] = 0x01U;
    txData[1] = diagnostic->sensor_id;
    txData[2] = diagnostic->error_code;
    txData[3] = diagnostic->consecutive_error_count;
    txData[4] = (uint8_t)(diagnostic->last_distance_mm & 0xFFU);
    txData[5] = (uint8_t)(diagnostic->last_distance_mm >> 8);
    txData[6] = diagnostic->status;
    txData[7] = diagnostic->sequence;

    txHeader.StdId = ROS_CAN_SENSOR_DIAG_ID;
    txHeader.ExtId = 0U;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8U;
    txHeader.TransmitGlobalTime = DISABLE;
    return CAN1_TrySend(&txHeader, txData);
  }

static HAL_StatusTypeDef CAN1_SendAck(uint16_t seq, uint8_t cmd, uint8_t result)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};

  txData[0] = (uint8_t)(seq >> 8);
  txData[1] = (uint8_t)(seq & 0xFFU);
  txData[2] = cmd;
  txData[3] = result;

  txHeader.StdId = ROS_CAN_ACK_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  return can_transport_send_reply(&hcan1, &txHeader, txData);
}

static void CAN1_SendStats(void)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  Motor_CommStats_t stats;

  motor_set_timeout_drop_count(zdt_can_driver_get_timeout_drop_count());
  motor_set_can2_tx_fail_count(zdt_can_driver_get_tx_fail_count());
  motor_get_comm_stats(&stats);
  stats.can1_tx_fail_count = can_transport_get_failure_count(&hcan1);

  txHeader.StdId = ROS_CAN_STATS_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = 0x01U;
  txData[1] = (uint8_t)(stats.rx_count & 0xFFU);
  txData[2] = (uint8_t)((stats.rx_count >> 8) & 0xFFU);
  txData[3] = (uint8_t)(stats.enqueue_drop_count & 0xFFU);
  txData[4] = (uint8_t)((stats.enqueue_drop_count >> 8) & 0xFFU);
  txData[5] = (uint8_t)(stats.timeout_drop_count & 0xFFU);
  txData[6] = (uint8_t)((stats.timeout_drop_count >> 8) & 0xFFU);
  txData[7] = 0U;
  (void)CAN1_TrySend(&txHeader, txData);

  txData[0] = 0x02U;
  txData[1] = (uint8_t)(stats.exec_ok_count & 0xFFU);
  txData[2] = (uint8_t)((stats.exec_ok_count >> 8) & 0xFFU);
  txData[3] = (uint8_t)(stats.exec_fail_count & 0xFFU);
  txData[4] = (uint8_t)((stats.exec_fail_count >> 8) & 0xFFU);
  txData[5] = (uint8_t)(stats.last_seq >> 8);
  txData[6] = (uint8_t)(stats.last_seq & 0xFFU);
  txData[7] = stats.last_result;
  (void)CAN1_TrySend(&txHeader, txData);

  txData[0] = 0x03U;
  txData[1] = (uint8_t)(stats.can1_tx_fail_count & 0xFFU);
  txData[2] = (uint8_t)((stats.can1_tx_fail_count >> 8) & 0xFFU);
  txData[3] = (uint8_t)(stats.can2_tx_fail_count & 0xFFU);
  txData[4] = (uint8_t)((stats.can2_tx_fail_count >> 8) & 0xFFU);
  txData[5] = (uint8_t)(stats.stop_fail_count & 0xFFU);
  txData[6] = (uint8_t)((stats.stop_fail_count >> 8) & 0xFFU);
  txData[7] = 0U;
  (void)CAN1_TrySend(&txHeader, txData);
}

static void CAN1_SendEmergencyEvent(uint8_t motor_idx, uint8_t reason_code, int32_t speed_rpm)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};

  txData[0] = 0x06U;
  txData[1] = motor_idx;
  txData[2] = reason_code;
  txData[3] = (uint8_t)(speed_rpm & 0xFF);
  txData[4] = (uint8_t)((uint32_t)speed_rpm >> 8);
  txData[5] = (uint8_t)((uint32_t)speed_rpm >> 16);
  txData[6] = (uint8_t)((uint32_t)speed_rpm >> 24);
  txData[7] = 0U;

  txHeader.StdId = ROS_CAN_STATUS_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  (void)can_transport_send_reply(&hcan1, &txHeader, txData);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

#if defined(STM32_DIAGNOSTIC_CAN_PROBE) && STM32_DIAGNOSTIC_CAN_PROBE
  /* Standalone probe: run before GPIO/PWM, CAN2, sensors and the scheduler. */
  extern void can_probe_run(void);
  can_probe_run();
  for (;;) { }
#endif

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (Servo_Init() != HAL_OK)
  {
    Error_Handler();
  }

  /* Three DS3230MG outputs start at calibrated logical zero. PC8/CH3
   * follows PC6/CH1; PC9/CH4 remains disabled. SG90 uses TIM3/TIM4. */
  if (ArmServo_Init() != HAL_OK)
  {
    Error_Handler();
  }

  CAN_Filter_Config();

  /* Sensor bring-up uses HAL_Delay and tick-based I2C timeouts. Initialize
   * before any FreeRTOS object allocation: pre-scheduler critical sections
   * can leave BASEPRI set and mask the TIM6 HAL tick until osKernelStart. */
  (void)distance_sensor_init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  can_transport_init(&hcan1, &hcan2);
  zdt_can_driver_init(&hcan2);
  motor_control_init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlTask_attributes);
  statusTaskHandle = osThreadNew(StartStatusTask, NULL, &statusTask_attributes);
  heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  if ((defaultTaskHandle == NULL) || (motorControlTaskHandle == NULL) ||
      (statusTaskHandle == NULL) || (heartbeatTaskHandle == NULL) || (sensorTaskHandle == NULL))
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* CAN startup and motor enable run from StartDefaultTask after the
   * scheduler owns the HAL tick.  HAL_CAN_Start() has a tick-based wait and
   * must not run in this pre-scheduler path. */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

#if defined(STM32_DIAGNOSTIC_HSE_DIRECT) && STM32_DIAGNOSTIC_HSE_DIRECT
  /* Clock-only measurement: HSE feeds SYSCLK directly. Even a 25 MHz
   * crystal stays within CPU limits. HAL assumes HSE_VALUE=8 MHz, so the
   * tick/wall-time ratio reveals the physical HSE frequency. No CAN/PWM. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
  return;
#endif

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
#if defined(STM32_DIAGNOSTIC_HSI_CLOCK) && STM32_DIAGNOSTIC_HSI_CLOCK
  /* Diagnostic only: 16 MHz HSI / 16 * 144 / 2 = 72 MHz, independent
   * of the external crystal. Keep all peripheral clock rates unchanged. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* Standard F407 board uses an 8 MHz HSE crystal (not the cam board's
   * 25 MHz crystal).  8 / 8 * 144 / 2 = 72 MHz. */
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  /* ROS telemetry must not retry forever when the peer cannot ACK. The
   * transport observes single-shot results and backs off on failures. */
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure the physical start button as an active-low input. */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure new-board K0/K1 pins as active-low inputs. */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Hold every VL53L1X in reset until individual addresses are assigned. */
  HAL_GPIO_WritePin(DISTANCE_SENSOR_XSHUT_GPIO_Port, DISTANCE_SENSOR_XSHUT_PINS, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = DISTANCE_SENSOR_XSHUT_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISTANCE_SENSOR_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static uint8_t Servo_DemoSetBoth(int16_t angle_deg)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (servo_remote_owned) {
    if (primask == 0U) __enable_irq();
    return 0U;
  }
  servo_return_pending = 0U;
  Servo_SetBoth(angle_deg);
  if (primask == 0U) __enable_irq();
  return 1U;
}

static void WkupDemo_Service(void)
{
  const uint32_t now = HAL_GetTick();
  if (wkup_demo_state != WKUP_DEMO_IDLE &&
      (!ArmServo_DemoActive() || servo_remote_owned)) {
    /* A CAN arm command cancelled the local sequence; preserve CAN ownership. */
    (void)ArmServo_DemoEnd();
    wkup_demo_state = WKUP_DEMO_IDLE;
    return;
  }

  switch (wkup_demo_state) {
    case WKUP_DEMO_IDLE:
      if (ArmServo_PollDemo(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (ArmServo_DemoSetAngle(ARM_SERVO_WK_PLACE_DEG)) {
          servo_return_pending = 0U;
          wkup_demo_stage_tick = now;
          wkup_demo_state = WKUP_DEMO_ARM_FORWARD;
        }
        if (primask == 0U) __enable_irq();
      }
      break;

    case WKUP_DEMO_ARM_FORWARD:
      if ((uint32_t)(now - wkup_demo_stage_tick) >= ARM_SERVO_WK_MOVE_MS + ARM_SERVO_WK_HOLD_MS) {
        if (Servo_DemoSetBoth(ARM_SERVO_WK_SERVO_ANGLE)) { /* open: 0 degrees */
          wkup_demo_stage_tick = now;
          wkup_demo_state = WKUP_DEMO_SERVO_OPEN;
        } else {
          (void)ArmServo_DemoEnd();
          wkup_demo_state = WKUP_DEMO_IDLE;
        }
      }
      break;

    case WKUP_DEMO_SERVO_OPEN:
      if ((uint32_t)(now - wkup_demo_stage_tick) >= ARM_SERVO_WK_SERVO_HOLD_MS) {
        if (Servo_DemoSetBoth(ARM_SERVO_WK_SERVO_RETURN_ANGLE)) { /* close: 180 degrees */
          wkup_demo_stage_tick = now;
          wkup_demo_state = WKUP_DEMO_SERVO_CLOSE;
        } else {
          (void)ArmServo_DemoEnd();
          wkup_demo_state = WKUP_DEMO_IDLE;
        }
      }
      break;

    case WKUP_DEMO_SERVO_CLOSE:
      if ((uint32_t)(now - wkup_demo_stage_tick) >= ARM_SERVO_WK_SERVO_RETURN_MS) {
        if (ArmServo_DemoSetAngle(ARM_SERVO_WK_HOME_DEG)) {
          wkup_demo_stage_tick = now;
          wkup_demo_state = WKUP_DEMO_ARM_RETURN;
        }
      }
      break;

    case WKUP_DEMO_ARM_RETURN:
      if ((uint32_t)(now - wkup_demo_stage_tick) >= ARM_SERVO_WK_MOVE_MS) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (ArmServo_DemoEnd()) {
          wkup_demo_state = WKUP_DEMO_IDLE;
        }
        if (primask == 0U) __enable_irq();
      }
      break;

    default:
      wkup_demo_state = WKUP_DEMO_IDLE;
      break;
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    CAN_ServiceStartup();
    if ((can1_started != 0U) && (can2_started != 0U) &&
        (can_motors_enabled == 0U))
    {
      if (motor_enable_all() != HAL_OK)
      {
        Error_Handler();
      }
      can_motors_enabled = 1U;
      motor_set_ros_alive();
    }
    osDelay(1000U);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
  * @brief  Pulls command queue and updates target registers every 10ms.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  Motor_Command_t cmd;
  HAL_StatusTypeDef cmd_result;
  uint32_t last_control_tick = HAL_GetTick() - CHASSIS_CONTROL_PERIOD_MS;

  for(;;)
  {
    /* Sample the physical arm keys from task context. TIM6 stays short and
     * continues generating PWM/HAL ticks while collision checks run. */
    Servo_PollButtons();
    WkupDemo_Service();
    uint32_t now = HAL_GetTick();
    if ((now - last_control_tick) >= CHASSIS_CONTROL_PERIOD_MS)
    {
      last_control_tick = now;
      Buttons_Service();
      motor_safety_poll();
      /* Drain a bounded burst, with interrupts enabled between commands.
       * Four chassis motors at 50 Hz already exceed the old 100 commands/s limit. */
      for (uint8_t issued = 0U; issued < MOTOR_CMD_QUEUE_LENGTH; issued++)
      {
        if (motor_process_next_command(&cmd, &cmd_result) == 0U) { break; }
        /* Transport retains replies independently of telemetry and records
         * overflow; a broken reply path must never block emergency stops. */
        (void)CAN1_SendAck(cmd.seq, cmd.cmd, (cmd_result == HAL_OK) ? 0U : 1U);
        if ((cmd.source == MOTOR_CMD_SOURCE_LOCAL_BUTTON) && (cmd_result != HAL_OK))
        {
          CAN1_SendButtonEvent(ROS_BUTTON_EVENT_START,
                               ROS_BUTTON_RESULT_EXECUTION_FAILED, cmd.seq);
        }
        can_transport_service();
      }
    }
    zdt_can_driver_timeout_poll();
    can_transport_service();
    osDelay(1U);
  }
}

/* USER CODE BEGIN Header_StartStatusTask */
/**
  * @brief  Polls status from drivers and reports to ROS every 20ms.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStatusTask */
void StartStatusTask(void *argument)
{
  uint8_t i;
  uint8_t stats_div = 0U;
  zdt_estop_event_t estop_event;

  for(;;)
  {
    motor_update_status(STATUS_QUERY_BUDGET_PER_CYCLE);
    for (i = 0U; i < MOTOR_COUNT; i++)
    {
      CAN1_SendStatus(i);
    }

    stats_div++;
    if (stats_div >= 4U)
    {
      stats_div = 0U;
      CAN1_SendStats();
    }

    if (zdt_status_take_estop_event(&estop_event) == 1U)
    {
      (void)motor_stop_all();
      CAN1_SendEmergencyEvent(estop_event.motor_idx, estop_event.status_flags, estop_event.speed_rpm);
    }

    osDelay(STATUS_TASK_PERIOD_MS);
  }
}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
  * @brief  Monitors ROS heartbeat and triggers stop on timeout.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  uint8_t timeout_latched = 0U;

  for(;;)
  {
    if (motor_is_ros_timeout(CHASSIS_HEARTBEAT_TIMEOUT_MS) == 1U)
    {
      if (timeout_latched == 0U)
      {
        (void)motor_stop_all();
        CAN1_SendEmergencyEvent(0xFFU, ROS_HEARTBEAT_TIMEOUT_FLAG, 0);
        timeout_latched = 1U;
      }
    }
    else
    {
      timeout_latched = 0U;
    }
    osDelay(20U);
  }
}

void StartSensorTask(void *argument)
{
  uint8_t i;
  distance_sensor_diag_t diagnostic;
  distance_sensor_sample_t pending_samples[DISTANCE_SENSOR_COUNT] = {0};
  distance_sensor_diag_t pending_diagnostics[DISTANCE_SENSOR_COUNT] = {0};
  uint8_t sample_pending[DISTANCE_SENSOR_COUNT] = {0};
  uint8_t diagnostic_pending[DISTANCE_SENSOR_COUNT] = {0};

  (void)argument;
  for (;;)
  {
    distance_sensor_poll();

    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++)
    {
      if (distance_sensor_take_updated(i, &pending_samples[i]) == 1U)
      {
        motor_update_distance(&pending_samples[i]);
        /* Replace unsent periodic data with the newest sample for this sensor. */
        sample_pending[i] = 1U;
      }
    }

    while (distance_sensor_take_diagnostic(&diagnostic) == 1U)
    {
      pending_diagnostics[diagnostic.sensor_id] = diagnostic;
      diagnostic_pending[diagnostic.sensor_id] = 1U;
    }

    /* Give one-shot errors the first available queue slots. Retain each
     * report until accepted; retries never wait for a hardware mailbox. */
    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++)
    {
      if ((diagnostic_pending[i] != 0U) &&
          (CAN1_SendSensorDiag(&pending_diagnostics[i]) == HAL_OK))
      {
        diagnostic_pending[i] = 0U;
      }
    }

    for (i = 0U; i < DISTANCE_SENSOR_COUNT; i++)
    {
      if ((sample_pending[i] != 0U) &&
          (CAN1_SendDistance(&pending_samples[i]) == HAL_OK))
      {
        sample_pending[i] = 0U;
      }
    }

    osDelay(DISTANCE_POLL_PERIOD_MS);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
    if (servo_initialized != 0U)
    {
      Servo_Service();
    }
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
