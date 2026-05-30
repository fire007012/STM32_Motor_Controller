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
#include "zdt_can_driver.h"
#include "motor_control.h"
#include "zdt_status.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATUS_TASK_PERIOD_MS              20U
#define STATUS_QUERY_BUDGET_PER_CYCLE      (MOTOR_COUNT * 3U)
#define ROS_HEARTBEAT_TIMEOUT_FLAG         0x80U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

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
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void StartStatusTask(void *argument);
void StartHeartbeatTask(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static HAL_StatusTypeDef CAN1_TrySend(const CAN_TxHeaderTypeDef *txHeader, uint8_t txData[8]);
static void CAN1_SendStatus(uint8_t motor_idx);
static void CAN1_SendAck(uint16_t seq, uint8_t cmd, uint8_t result);
static void CAN1_SendStats(void);
static void CAN1_SendEmergencyEvent(uint8_t motor_idx, uint8_t reason_code, int32_t speed_rpm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

static HAL_StatusTypeDef CAN1_TrySend(const CAN_TxHeaderTypeDef *txHeader, uint8_t txData[8])
{
  uint32_t txMailbox;
  HAL_StatusTypeDef status;

  status = HAL_CAN_AddTxMessage(&hcan1, (CAN_TxHeaderTypeDef *)txHeader, txData, &txMailbox);
  if (status != HAL_OK)
  {
    motor_record_can1_tx_failure();
  }

  return status;
}

static void CAN1_SendStatus(uint8_t motor_idx)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  int32_t pos;
  int32_t vel;
  uint8_t report_mask;

  if (motor_idx >= MOTOR_COUNT)
  {
    return;
  }

  txData[0] = 0x01U;
  txData[1] = motor_idx;
  txData[2] = (uint8_t)(motors[motor_idx].status_word & 0xFFU);
  txData[3] = (uint8_t)(motors[motor_idx].status_word >> 8);
  txData[4] = (uint8_t)(motors[motor_idx].fault_code & 0xFFU);
  txData[5] = (uint8_t)(motors[motor_idx].fault_code >> 8);
  txData[6] = motors[motor_idx].fault_flag;
  txData[7] = 0U;

  txHeader.StdId = ROS_CAN_STATUS_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8U;
  txHeader.TransmitGlobalTime = DISABLE;

  report_mask = motor_get_report_mask();

  if ((report_mask & MOTOR_REPORT_BASIC) != 0U)
  {
    (void)CAN1_TrySend(&txHeader, txData);
  }

  pos = motors[motor_idx].position_feedback;
  txData[0] = 0x02U;
  txData[1] = motor_idx;
  txData[2] = (uint8_t)(pos & 0xFF);
  txData[3] = (uint8_t)((uint32_t)pos >> 8);
  txData[4] = (uint8_t)((uint32_t)pos >> 16);
  txData[5] = (uint8_t)((uint32_t)pos >> 24);
  txData[6] = 0U;
  txData[7] = 0U;

  if ((report_mask & MOTOR_REPORT_POSITION) != 0U)
  {
    (void)CAN1_TrySend(&txHeader, txData);
  }

  vel = motors[motor_idx].current_velocity;
  txData[0] = 0x03U;
  txData[1] = motor_idx;
  txData[2] = (uint8_t)(vel & 0xFF);
  txData[3] = (uint8_t)((uint32_t)vel >> 8);
  txData[4] = (uint8_t)((uint32_t)vel >> 16);
  txData[5] = (uint8_t)((uint32_t)vel >> 24);
  txData[6] = 0U;
  txData[7] = 0U;

  if ((report_mask & MOTOR_REPORT_VELOCITY) != 0U)
  {
    (void)CAN1_TrySend(&txHeader, txData);
  }

  txData[0] = 0x04U;
  txData[1] = motor_idx;
  txData[2] = (uint8_t)(motors[motor_idx].target_velocity & 0xFF);
  txData[3] = (uint8_t)((uint32_t)motors[motor_idx].target_velocity >> 8);
  txData[4] = (uint8_t)((uint32_t)motors[motor_idx].target_velocity >> 16);
  txData[5] = (uint8_t)((uint32_t)motors[motor_idx].target_velocity >> 24);
  txData[6] = 0U;
  txData[7] = 0U;

  if ((report_mask & MOTOR_REPORT_TARGET) != 0U)
  {
    (void)CAN1_TrySend(&txHeader, txData);

    txData[0] = 0x05U;
    txData[1] = motor_idx;
    txData[2] = (uint8_t)(motors[motor_idx].target_position & 0xFF);
    txData[3] = (uint8_t)((uint32_t)motors[motor_idx].target_position >> 8);
    txData[4] = (uint8_t)((uint32_t)motors[motor_idx].target_position >> 16);
    txData[5] = (uint8_t)((uint32_t)motors[motor_idx].target_position >> 24);
    txData[6] = 0U;
    txData[7] = 0U;
    (void)CAN1_TrySend(&txHeader, txData);
  }
}

static void CAN1_SendAck(uint16_t seq, uint8_t cmd, uint8_t result)
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

  (void)CAN1_TrySend(&txHeader, txData);
}

static void CAN1_SendStats(void)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  Motor_CommStats_t stats;

  motor_set_timeout_drop_count(zdt_can_driver_get_timeout_drop_count());
  motor_set_can2_tx_fail_count(zdt_can_driver_get_tx_fail_count());
  motor_get_comm_stats(&stats);

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

  (void)CAN1_TrySend(&txHeader, txData);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* ── K0/K1 测试代码已注释, 恢复 FreeRTOS 正常启动 ── */
#if 0
  /* ── K0(PE4)=逆时针180°(CCW) / K1(PE3)=顺时针180°(CW) ── */
  {
    ... (button test code preserved but disabled) ...
  }
#endif

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  zdt_can_driver_init(&hcan2);
  motor_control_init();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlTask_attributes);
  statusTaskHandle = osThreadNew(StartStatusTask, NULL, &statusTask_attributes);
  heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
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
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
    osDelay(1);
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

  for(;;)
  {
    if (motor_fetch_command(&cmd, 0U) == 1U)
    {
      cmd_result = motor_apply_command(&cmd);
      CAN1_SendAck(cmd.seq, cmd.cmd, (cmd_result == HAL_OK) ? 0U : 1U);
    }
    zdt_can_driver_timeout_poll();
    osDelay(10U);
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
    if (motor_is_ros_timeout(500U) == 1U)
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
