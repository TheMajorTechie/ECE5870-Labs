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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PrepareI2C2Transaction(uint32_t address, char RD_WRN, int numbytes) {
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));	//clear NBYTES and ADD bitfields
	
	//set # of bytes to transmit=numbytes, device address 0x69, RD_WRN to write, and start
	I2C2->CR2 |= ((numbytes << 16) | (address << 1));
	
	if(RD_WRN == 'w')						//request a write
		I2C2->CR2 &= ~(1 << 10);
	else if(RD_WRN == 'r')			//request a read
		I2C2->CR2 |= (1 << 10);
	else												//assume a write by default
		I2C2->CR2 &= ~(1 << 10);
	
	//start
	I2C2->CR2 |= (1 << 13);
	
	return;
}

void TransmissionWriteHelper(uint32_t address, int numbytes, uint32_t data) {
	
		PrepareI2C2Transaction(address, 'w', numbytes);
	
		//transmission block
		while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF));
		if(I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = data;
		}
		else if(I2C2->ISR & I2C_ISR_NACKF) {
			GPIOC->ODR |= ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));			//turn on all LEDs for fail
		}
}

char TransmissionReadHelper(uint32_t address, int numbytes) {
	PrepareI2C2Transaction(address, 'r', numbytes);
	while(!(I2C2->ISR & I2C_ISR_RXNE) & !(I2C2->ISR & I2C_ISR_NACKF));
	if(I2C2->ISR & I2C_ISR_RXNE) {
		return I2C2->RXDR;
	}
	else if(I2C2->ISR & I2C_ISR_NACKF) {
		GPIOC->ODR |= ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));			//turn on all LEDs for fail
	}
	return -1;	//return an error
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
	SystemClock_Config();

	//enable GPIOB and GPIOC in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//set PB11 to alternate function mode, open-drain output type, and I2C2_SDA as alt funct
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER11)) | GPIO_MODER_MODER11_1);
	GPIOB->OTYPER |= (1 << 11);	//set bit 11 in OTYPER to open-drain
	GPIOB->AFR[1] |= (1 << 12);	//we want alternate function mode AF1 for AFSEL11 (pin 11 on port b)
	
	//set PB13 to alternate function mode, open-drain output type, and I2C2_SCL as alt function
	GPIOB->MODER = (GPIOB->MODER & (~(GPIO_MODER_MODER13)) | GPIO_MODER_MODER13_1);
	GPIOB->OTYPER |= (1 << 13);	
	GPIOB->AFR[1] |= (0x5 << 20);	//AF5 on AFSEL13 selected
	
	//set PB14 to output mode, push-pull output, and initialize/set pin high
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1<<14);
	GPIOB->ODR |= (1 << 14);
	
	//set PB15 to input mode
	GPIOB->MODER = (GPIOB->MODER & ~((GPIO_MODER_MODER15) | (GPIO_MODER_MODER15_1)));
	
	//set PC0 to output mode, push-pull output, and initialize/set pin high
	GPIOC->MODER |= 1;
	GPIOC->OTYPER &= ~(1);
	GPIOC->ODR |= 1;
	
	//prepare red, blue, orange, & green LEDs
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
	
	//enable the I2C2 peripheral's system clock in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//now set the parameters in the TIMINGR register to use 100kHz standard mode I2C
	//PRESC=1, SCLDEL=0x4, SDADEL=0x2, SCLH=0xF, SCLL=0x13
	I2C2->TIMINGR |= ((1 << 28) | (0x4 << 20) | (0x2 << 16) | (0xf << 8) | (0x13));
	
	//enable I2C2 using PE bit in CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
	
	//set transaction params in CR2 register
	/*	SADD[7:1]=slave address
			NBYTES[7:0]=# of data bytes to be transmitted
			RD_WRN=read/write
			don't set AUTOEND bit
			set START bit to begin address frame (do this LAST)
	*/
	
	/******NOTES ON GYRO DEVICE
	Device address is 0x69 instead of 0x6B
	Value in who_am_i register is 0xD3 instead of 0xD4
	*/
	
	//call a helper method to set up a transaction in write mode with the proper address
	PrepareI2C2Transaction(0x69, 'w', 1);			
	
//	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));	//clear NBYTES and ADD bitfields
//	
//	//set # of bytes to transmit=1, device address 0x69, RD_WRN to write, and start
//	I2C2->CR2 |= ((1 << 16) | (0x69 << 1));
//	//request a write
//	I2C2->CR2 &= ~(1 << 10);
//	
//	//start
//	I2C2->CR2 |= (1 << 13);
	
	//wait until either TXIS or NACKF flag set. if NACKF, this is an error state. 
	while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF)) 
		;
	
	if(I2C2->ISR & I2C_ISR_TXIS) {	//check for TXIS bit set
		GPIOC->ODR |= (1 << 7);			//set blue LED
	}
	else if(I2C2->ISR & I2C_ISR_NACKF) {
		GPIOC->ODR |= (1 << 6);			//set red LED
	}
	else {
		GPIOC->ODR |=	(1 << 9);			//set green LED
		GPIOC->ODR &= ~((1 << 6) | (1 << 7));	//clear both LEDs
	}
		
	//write the address of the "WHO_AM_I" register into I2C transmit register
	I2C2->TXDR |= (0xF);
	
	while(!(I2C2->ISR & I2C_ISR_TC)) 
		;
	
	PrepareI2C2Transaction(0x69, 'r', 1);
	

	//wait until either RXNE or NACKF
	while(!(I2C2->ISR & I2C_ISR_RXNE) & !(I2C2->ISR & I2C_ISR_NACKF))
		;
	
	if(I2C2->ISR & I2C_ISR_RXNE) {
		GPIOC->ODR |= (1 << 8);			//set orange LED
	}
	else if(I2C2->ISR & I2C_ISR_NACKF) {
		GPIOC->ODR |= (1 << 6);			//set red LED
	}
	else {
		GPIOC->ODR |=	(1 << 9);			//set green LED
		GPIOC->ODR &= ~((1 << 6) | (1 << 8));	//clear both LEDs
	}
	
	//if the transfer complete flag is set and the register contents
	//matches the expected 0xD3 value, then turn off the blue LED
	if(I2C2->ISR & I2C_ISR_RXNE) {
		while(!(I2C2->ISR & I2C_ISR_TC))
			;
		if(I2C2->ISR & I2C_ISR_TC) {
			if(I2C2->RXDR == 0xd3) {
				I2C2->CR2 |= (1 << 14);		//SET the stop bit in CR2 to release I2C bus
				GPIOC->ODR &= ~(1 << 7);
			}
		}
	}
	
	
	
//part 2----------------------------------------------------------	
	//enable the X and Y sensing axes in the CTRL_REG1 register
	PrepareI2C2Transaction(0x69, 'w', 2);
	
	//wait for TXIS or NACKF flags
	while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF)) 
		;
	
	if(I2C2->ISR & I2C_ISR_TXIS) {	//check for TXIS bit set
		GPIOC->ODR |= (1 << 7);			//set blue LED for success
		I2C2->TXDR = 0x20;					//if success, write CTRL_REG1's address to TXDR
	}
	else if(I2C2->ISR & I2C_ISR_NACKF) {
		GPIOC->ODR |= (1 << 6);			//set red LED for fail
	}
	else {
		GPIOC->ODR |=	(1 << 9);			//set green LED
		GPIOC->ODR &= ~((1 << 6) | (1 << 7));	//clear both LEDs
	}
	
	while(!(I2C2->ISR & I2C_ISR_TXIS) & !(I2C2->ISR & I2C_ISR_NACKF)) 
		;	//wait for flags again
	if(I2C2->ISR & I2C_ISR_TXIS) {	//check for TXIS bit set
		GPIOC->ODR |= (1 << 7);			//set blue LED for success
		I2C2->TXDR = 0x0B;					//if success, write 1011 to CTRL_REG1 to set normal power mode, Xen and Yen
	}
	
	//now wait for a transmit complete
	while(!(I2C2->ISR & I2C_ISR_TC)) 
		;
	
	uint8_t x1, x2, y1, y2;		//SET up the partial x and y values for storage
	int16_t x, y, x_dir, y_dir;		//complete x and y with direction
	
	
	
	
  while (1)
  {
		//clear all LEDs
		GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
		
		//read x1
		TransmissionWriteHelper(0x69, 1, 0x28);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		x1 = TransmissionReadHelper(0x69, 1);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		
		//read x2
		TransmissionWriteHelper(0x69, 1, 0x29);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		x2 = TransmissionReadHelper(0x69, 1);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		
		x = (x1 | x2 << 8);
		x_dir += x;
		
		//read y1
		TransmissionWriteHelper(0x69, 1, 0x2a);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		y1 = TransmissionReadHelper(0x69, 1);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		
		//read y2
		TransmissionWriteHelper(0x69, 1, 0x2b);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		y2 = TransmissionReadHelper(0x69, 1);
		while(!(I2C2->ISR & I2C_ISR_TC));	//wait for TC
		
		y = (y1 | y2 << 8);
		y_dir += y;
		
		
		
		
		//orange & green
		if(x_dir < 0) {
			GPIOC->ODR |= (1 << 8);		//enable orange
			GPIOC->ODR &= ~(1 << 9);	//disable green
		}
		else {
			GPIOC->ODR &= ~(1 << 8);	//disable orange
			GPIOC->ODR |= (1 << 9);		//enable green
		}
		
		//red & blue
		if(y_dir < 0) {
			GPIOC->ODR |= (1 << 6);		//enable red
			GPIOC->ODR &= ~(1 << 7);	//disable blue
		}
		else {
			GPIOC->ODR &= ~(1 << 6);	//disable red
			GPIOC->ODR |= (1 << 7);		//enable blue
		}
		
		HAL_Delay(100);	//delay 100ms
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
