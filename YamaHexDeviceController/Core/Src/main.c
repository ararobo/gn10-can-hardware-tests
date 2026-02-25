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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_DUTY_MIN_PERCENT (-100)
#define MOTOR_DUTY_MAX_PERCENT (100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void initCAN(void);
HAL_StatusTypeDef sendPacket(uint16_t stdId, const uint8_t *data, uint8_t len);
void Motor_SetDutyPercent(int8_t duty_percent);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef RxFilter;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t RxData[8];
bool RxFlag;

void initCAN(void)
{
  RxFilter.FilterBank = 0;
  RxFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  RxFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  RxFilter.FilterIdHigh = 0x0000;
  RxFilter.FilterIdLow = 0x0000;
  RxFilter.FilterMaskIdHigh = 0x0000;
  RxFilter.FilterMaskIdLow = 0x0000;
  RxFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  RxFilter.FilterActivation = ENABLE;
  RxFilter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &RxFilter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}
void Motor_SetDutyPercent(int8_t duty_percent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;
  uint32_t pulse = 0U;
  int32_t limited = duty_percent;

  if (limited > MOTOR_DUTY_MAX_PERCENT)
  {
    limited = MOTOR_DUTY_MAX_PERCENT;
  }
  else if (limited < MOTOR_DUTY_MIN_PERCENT)
  {
    limited = MOTOR_DUTY_MIN_PERCENT;
  }

  if (limited >= 0)
  {
    pulse = (uint32_t)((limited * (int32_t)arr) / MOTOR_DUTY_MAX_PERCENT);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  }
  else
  {
    pulse = (uint32_t)(((-limited) * (int32_t)arr) / MOTOR_DUTY_MAX_PERCENT);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
  }
}

HAL_StatusTypeDef sendPacket(uint16_t stdId, const uint8_t *data, uint8_t data_length)
{
  if (data_length > 8)
  {
    return 0;
  }

  TxHeader.StdId = stdId;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = data_length;
  TxHeader.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(&hcan, &TxHeader, (uint8_t *)data, &TxMailbox);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  initCAN();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Motor_SetDutyPercent(50);
  char b = '\'';
  HAL_UART_Transmit(&huart1, (uint8_t *)&b, 1, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t tx_data[1] = {1};
    sendPacket(0x123, tx_data, sizeof(tx_data));
    HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    // c言語のptrintfっぽいのをuartで垂れ流す
    HAL_UART_Transmit(&huart1, RxData, 8, 1000); // UARTにRxで受信したデータを流す
    char a = '\n';
    HAL_UART_Transmit(&huart1, (uint8_t *)&a, 1, 1000); // そのままだと改行がないためどこがはじめでどこが最後かわからなくなってしまうので改行をa(char)で垂れ流す。
  }
}

/* USER CODE END 4 */

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
