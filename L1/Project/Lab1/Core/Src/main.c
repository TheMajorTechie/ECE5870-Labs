/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
int main(void) {
	SystemClock_Config(); //Configure the system clock

	//enable GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
	
	/*
•	Configure LED pins on GPIOC (PC6-PC9) in the following way:
	– General-purpose output mode using the MODER register.
	– Push-pull output type using the OTYPER register.
	– Low speed using the OSPEEDR register.
	– No pull-up/down resistors using the PUPDR register.
	6 = red | 7 = blue | 8 = orange | 9 = green
	*/
	
	//set the lower bits for PC6/7/8/9 in moder register for general-purpose output mode
	GPIOC->MODER |= ((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	
	//clear the bits for output type to put into push-pull
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	
	//clear the lower bits for the output speed register to put into low speed
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	
	//clear both upper and lower bits to set no pull-ups or pull-downs
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | 
										(1 << 14) | (1 << 15) | 
										(1 << 16) | (1 << 17) |
										(1 << 18) | (1 << 19)
										);
										
	//set red LED to 1 and blue LED to 0
	GPIOC->ODR |= (1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	
	/*
	• USER Button pin (PA0) should be configured to:
	– Digital input mode using the MODER register. (floating internal pull-up/down)
	– Low speed using the OSPEEDR register.
	– Pull-down resistor using the PUPDR register	
	*/
	
	GPIOA->MODER &= ~((1) | (1 << 1));		//is it safe to assume that this will already be 00 out of reset?
	GPIOA->OSPEEDR &= ~(1);
	GPIOA->PUPDR |= (1 << 1);
	
	uint32_t debouncer = 0;
	while (1) {
		HAL_Delay(200); // Delay 200ms
		
//		debouncer = (debouncer << 1); // Always shift every loop iteration
//		
//		if (GPIOA->IDR & 0x1) { // If input signal is set/high debouncer |= 0x01; // Set lowest bit of bit-vector
//			debouncer |= 0x1;
//		}
//		if (debouncer == 0xFFFFFFFF) {
//		// This code triggers repeatedly when button is steady high!
//		}
//		if (debouncer == 0x00000000) {
//		// This code triggers repeatedly when button is steady low!
//		}
//		if (debouncer == 0x7FFFFFFF) {
		// This code triggers only once when transitioning to steady high!
			GPIOC->ODR ^= (1 << 6);
			GPIOC->ODR ^= (1 << 7);
//		}		
	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
