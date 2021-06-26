/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_rcc_ex.h>
#include <stm32f1xx_hal_tim.h>
#include <stm32f1xx_hal_tim_ex.h>
#include <stm32f1xx_hal_uart.h>
#include <sys/_stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int sec = 0;
int min = 0;
int hr = 0;
int msec = 0;
int sec2 = 0;
int min2 = 0;
int msec2 = 0;
_Bool m4 = false;
char buf[50];
int buf_len = 50;
int fbutton = 0;
int mode = 0;
_Bool checkval = 1;
_Bool checkval2 = 1;
_Bool BL = true;
//uint16_t count22=0;
int fbuttonpress12s = 0;
int fbutton2;
int count = 0;
//char
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin BT2_Pin */
  GPIO_InitStruct.Pin = B2_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Normal_clk(void){

	msec++;
		if(msec > 99){
			msec = 0;
			sec ++;
			if(sec > 59){
				sec = 0 ;
				min++;
				if(min > 59){
					min = 0;
					hr ++;
					if(hr > 23){
						hr = 0;
						min = 0;
						sec = 0;
						msec = 0;
					}
				}
			}
		}
}
void Readbutton(void) {

	if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) && checkval){
		if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
				//fbutton = 1;
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				//HAL_UART_Transmit(&huart2, "t", 4, 100);
				mode++;
				if(mode > 4)
				{
					mode = 0;
				}
				if(mode == 4)
				{
					msec2 =0 ;
					sec2 = 0;
					min2 = 0;
					m4=false;
				}

				checkval = false;
		}
	}else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET && checkval == false){
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				checkval = true;
				//fbutton = 0;
			}
		}
	}
void mode_zero(void){
	Normal_clk();
}
void mode_one(void){
	//Normal_clk();
	if(fbutton2 == 1 && fbuttonpress12s == 0){
		hr++;
		if(hr >23){
			hr=0;
		}
	}
	if(fbutton2 == 1 && fbuttonpress12s == 1){
			hr++;
			if(hr >23){
				hr=0;
			}
		}
}
void mode_two(void){
	if(fbutton2 == 1 && fbuttonpress12s == 0){
		min++;
		if(min > 59){
			min = 0;
			hr++;
		}
	}
	if(fbutton2 == 1 && fbuttonpress12s == 1){
		min++;
		if(min >59){
			min = 0;
			hr++;
		}
		}
}
void mode_three(void){
	if(fbutton2 == 1 && fbuttonpress12s == 0){
		sec++;
		if(sec > 59){
			min++;
			if(min > 59){
				hr++;
				if(hr > 23){
					min = 0;
					hr = 0;
					sec = 0;
				}
			}
		}
	}
	if(fbutton2 == 1 && fbuttonpress12s == 1){
		sec++;
		if(sec > 59){
			min++;
			if(min > 59){
				hr++;
				if(hr > 23){
					min = 0;
					hr = 0;
					sec = 0;
				}
			}
		}
	}
}
void mode_four(void){
	if(m4 == true){

	msec2++;
		if(msec2 > 99){
			msec2 = 0;
			sec2 ++;
			if(sec2 > 59){
				sec2 = 0 ;
				min2++;
				if(min2 > 59){
					min2 = 0;
					sec2 = 0;
					msec2 = 0;
				}
			}
		}
	}
}
enum {STATE13, STATE23, STATE33, STATE43} eState3;
void Readbutton2(void) {
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)){
		fbutton2 = 1;
		count++;
		if(count == 20){
			fbuttonpress12s = 1;
			switch(mode){
						case 0:
							mode_zero();
						case 1:
							//Normal_clk();

							mode_one();
							break;
						case 2:
							mode_two();
							break;
						case 3:
							mode_three();
							break;
							//Normal_clk();
						}
			count=0;
		}
	}
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) && checkval2){
		if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			fbutton2 = 1;
			//m4 = true;
			if(m4){
				m4 = false;
			}
			else{
				m4 = true;
			}
			switch(mode){
			case 1:
				mode_one();
				break;
			case 2:
				mode_two();
				break;
			case 3:
				mode_three();
				break;

				//Normal_clk();
			}
				checkval2 = 0;
		}
	}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && checkval2 == false){
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				fbuttonpress12s = 0;
				//count12 = 0;
				count = 0;
				checkval2 = 1;
				fbutton2 = 0;
				//m4 = 0;
			}
		}
}

//enum {STATE12, STATE22, STATE32, STATE42} eState2;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *time){
	if(mode != 4){
		Normal_clk();
		print_to_ter(msec,sec,min,hr);
	}else{
		 mode_four();
		print_to_ter(msec2,sec2,min2,hr);
	}

	Readbutton();
	Readbutton2();
}
void print_to_ter(int a,int b,int c,int d){
	switch(mode){
	case 0:
		buf_len = sprintf(buf, "\033[2J]	");
		buf_len = sprintf(buf, "mode%u %u:%u:%u      \r",mode,d,c,b);
		HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
		break;
	case 1:
		if(BL){
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u:%u:%u      \r",mode,d,c,b);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = false;
		}
		else{
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u  :%u:%u      \r",mode,c,b);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = true;
		}
		break;
	case 2:
		if(BL){
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u:%u:%u      \r",mode,d,c,b);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = false;
		}
		else{
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u: :%u      \r",mode,d,b);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = true;
		}
		break;
	case 3:
		if(BL){
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u:%u:%u      \r",mode,d,c,b);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = false;
		}
		else{
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u:%u:       \r",mode,d,c);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			BL = true;
		}
		break;
	case 4:
			buf_len = sprintf(buf, "\033[2J]	");
			buf_len = sprintf(buf, "mode%u %u:%u:%u      \r",mode,c,b,a);
			HAL_UART_Transmit(&huart2, (uint8_t *) buf, buf_len, 100);
			break;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
