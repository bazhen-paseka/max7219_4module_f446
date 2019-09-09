/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "max7219_4x_dot_sm.h"

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

/* USER CODE BEGIN PV */

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
	uint8_t digit[10][8];
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	max7219_struct h1_max7219 =
	{
		.spi		= &hspi1,
		.cs_port	= WriteStrob_GPIO_Port,
		.cs_pin		= WriteStrob_Pin
	};

	max7219_map h2_max7219_map;

	max7219_init(&h1_max7219, NoDecode, Intensity_3, DisplayDigit_0_7, NormalOperation);

	digit[0][7] = 0b00011111 ;
	digit[0][6] = 0b00010001 ;
	digit[0][5] = 0b00010001 ;
	digit[0][4] = 0b00010001 ;
	digit[0][3] = 0b00010001 ;
	digit[0][2] = 0b00010001 ;
	digit[0][1] = 0b00010001 ;
	digit[0][0] = 0b00011111 ;

	digit[1][7] = 0b00000100 ;
	digit[1][6] = 0b00001100 ;
	digit[1][5] = 0b00010100 ;
	digit[1][4] = 0b00000100 ;
	digit[1][3] = 0b00000100 ;
	digit[1][2] = 0b00000100 ;
	digit[1][1] = 0b00000100 ;
	digit[1][0] = 0b00011111 ;

	digit[2][7] = 0b00011111 ;
	digit[2][6] = 0b00000001 ;
	digit[2][5] = 0b00000001 ;
	digit[2][4] = 0b00011111 ;
	digit[2][3] = 0b00010000 ;
	digit[2][2] = 0b00010000 ;
	digit[2][1] = 0b00010000 ;
	digit[2][0] = 0b00011111 ;

	digit[3][7] = 0b00011111 ;
	digit[3][6] = 0b00000001 ;
	digit[3][5] = 0b00000001 ;
	digit[3][4] = 0b00011111 ;
	digit[3][3] = 0b00000001 ;
	digit[3][2] = 0b00000001 ;
	digit[3][1] = 0b00000001 ;
	digit[3][0] = 0b00011111 ;

	digit[4][7] = 0b00010001 ;
	digit[4][6] = 0b00010001 ;
	digit[4][5] = 0b00010001 ;
	digit[4][4] = 0b00011111 ;
	digit[4][3] = 0b00000001 ;
	digit[4][2] = 0b00000001 ;
	digit[4][1] = 0b00000001 ;
	digit[4][0] = 0b00000001 ;

	digit[5][7] = 0b00011111 ;
	digit[5][6] = 0b00010000 ;
	digit[5][5] = 0b00010000 ;
	digit[5][4] = 0b00011111 ;
	digit[5][3] = 0b00000001 ;
	digit[5][2] = 0b00000001 ;
	digit[5][1] = 0b00000001 ;
	digit[5][0] = 0b00011111 ;

	digit[6][7] = 0b00011111 ;
	digit[6][6] = 0b00010000 ;
	digit[6][5] = 0b00010000 ;
	digit[6][4] = 0b00011111 ;
	digit[6][3] = 0b00010001 ;
	digit[6][2] = 0b00010001 ;
	digit[6][1] = 0b00010001 ;
	digit[6][0] = 0b00011111 ;

	digit[7][7] = 0b00011111 ;
	digit[7][6] = 0b00000001 ;
	digit[7][5] = 0b00000001 ;
	digit[7][4] = 0b00000001 ;
	digit[7][3] = 0b00000001 ;
	digit[7][2] = 0b00000001 ;
	digit[7][1] = 0b00000001 ;
	digit[7][0] = 0b00000001 ;

	digit[8][7] = 0b00011111 ;
	digit[8][6] = 0b00010001 ;
	digit[8][5] = 0b00010001 ;
	digit[8][4] = 0b00011111 ;
	digit[8][3] = 0b00010001 ;
	digit[8][2] = 0b00010001 ;
	digit[8][1] = 0b00010001 ;
	digit[8][0] = 0b00011111 ;

	digit[9][7] = 0b00011111 ;
	digit[9][6] = 0b00010001 ;
	digit[9][5] = 0b00010001 ;
	digit[9][4] = 0b00011111 ;
	digit[9][3] = 0b00000001 ;
	digit[9][2] = 0b00000001 ;
	digit[9][1] = 0b00000001 ;
	digit[9][0] = 0b00011111 ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		for (uint8_t i=0; i<8; i++)
//		{
//			h2_max7219_map.kub_0[i] = 0 ;
//			h2_max7219_map.kub_1[i] = 0 ;
//			h2_max7219_map.kub_2[i] = 0 ;
//			h2_max7219_map.kub_3[i] = 0 ;
//		}

		for (uint8_t i=0; i<8; i++)
		{
		h2_max7219_map.kub_0[i] = digit[1][i]<<2 ;
		h2_max7219_map.kub_1[i] = digit[2][i]<<3 ;
		h2_max7219_map.kub_2[i] = digit[3][i]<<0 ;
		h2_max7219_map.kub_3[i] = digit[5][i]<<1 ;
		}

		max7219_show_all(&h1_max7219, h2_max7219_map);
		HAL_Delay(2000);

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

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
