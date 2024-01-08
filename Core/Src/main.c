/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#define BOARD_ID 1

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMITER
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern module_cfg configs;
bool aux = 0;
uint16_t adc_count = 0;
uint16_t sensorData[4][200] = {0};
uint16_t adcint[4] = {0};
float adc_sum[4] = {0};
uint16_t adc[4] = {0};
extern uint32_t TxMailbox;
uint8_t canRX[8] = {};
uint8_t canTX[8] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

	HAL_UART_Receive_DMA(&huart1, canRX, 8);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, &adcint, 4);


//	  uint8_t tx[8] = {1,225,67,68,69,70,71,72};
//	  TxHeader.StdId             = 0x0;     // ID do dispositivo
//	  TxHeader.RTR               = CAN_RTR_DATA;       //(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
//	  TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended
//	  TxHeader.DLC               = 8;      //Tamanho do pacote 0 - 8 bytes
//	  TxHeader.TransmitGlobalTime = DISABLE;
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (aux)
	  {
		  memset(&adc, 0, sizeof(adc));
		  for (int i = 0; i < 200; i++)
		  {
		      for (int j = 0; j < 4; j++)
		      {
		          if (configs.sensors[j].enable)
		          {
		              adc_sum[j] += sensorData[j][i] * 0.005;
		          }
		      }
		  }

		  for (int j = 0; j < 4; j++)
		  {
			  if (configs.sensors[j].enable)
			  {
				  adc[j] = (uint16_t)adc_sum[j];
			  }
		  }
		  send_sensor_data(adc);
		  aux = 0;
	  }


	#ifdef TRANSMITER
//	HAL_Delay(2000);
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx, &TxMailbox))
//	{
//		Error_Handler();
//	}

	#else

	#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if(htim->Instance==TIM2)
//	{
//		if (adc_count < 199)
//		{
//			sensorData[0][adc_count] = adcint[0];
//			sensorData[1][adc_count] = adcint[1];
////			sensorData[2][adc_count] = adcint[2];
////			sensorData[3][adc_count] = adcint[3];
//			adc_count++;
//		}
//		else
//		{
//			aux = 1;
//			adc_count = 0;
//		}
//	}
}

void fill_data(CanPacket *message, uint16_t adc, uint8_t pos, uint8_t sensor)
{
	message->packet.data[CAN_HEADER + 3*pos] = sensor;
	message->packet.data[CAN_HEADER + 3*pos + 1] = (uint8_t)(adc && 0xff00) >> 8;
	message->packet.data[CAN_HEADER + 3*pos + 2] = (uint8_t)(adc && 0x00ff);
}

void send_sensor_data(uint16_t *adc)
{
	uint8_t count = 0;
	CanPacket message = {0};
	for (int i = 0; i < SENSOR_NUMBERS ; i++)
	{
		if(configs.sensors[i].enable)
		{
			message.packet.ctrl0.control = configs.boardID;
			message.packet.ctrl1.control = 0; //revisar
			fill_data(&message, adc[i], count, i);

			TxHeader.StdId             = DATA;     // ID do dispositivo
			TxHeader.RTR               = CAN_RTR_DATA;       //(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
			TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended
			TxHeader.DLC               = 8;      //Tamanho do pacote 0 - 8 bytes
			TxHeader.TransmitGlobalTime = DISABLE;
			count++;
		}
		if (count == 2)
		{
			if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, message.packet.data, &TxMailbox))
			{
				Error_Handler();
			}
			memset(&message, 0, sizeof(CanPacket));
			count = 0;
		}
		else if ( (count == 1) && (i == (SENSOR_NUMBERS - 1) ) )
		{
			fill_data(&message, 0xffff, count, 2);

			if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, message.packet.data, &TxMailbox))
			{
				Error_Handler();
			}
		}
	}
}




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifdef TRANSMITER

#else
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	UartPacket uartPacket = {0};

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, canRX);

	DecodeCanPacket(RxHeader.StdId, &uartPacket, canRX);

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
#endif

}

void sendCanMsg_test(int delay)
{
	  uint8_t tx[7] = {1,2,3,4,5,6,7};
	  TxHeader.StdId             = 0x0;     // ID do dispositivo
	  TxHeader.RTR               = CAN_RTR_DATA;       //(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
	  TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended
	  TxHeader.DLC               = 7;      //Tamanho do pacote 0 - 8 bytes
	  TxHeader.TransmitGlobalTime = DISABLE;

	  int status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx, &TxMailbox);
	  if(status)
	  {
		 Error_Handler();
	  }
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(delay);
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

#ifdef  USE_FULL_ASSERT
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
