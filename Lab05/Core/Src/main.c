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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void I2C_WriteRegister(uint16_t deviceAddr, uint16_t data);

uint8_t I2C_ReadRegister(uint16_t deviceAddr);

void Enable_Ports();

void Init_Ports();

void I2C_SetUp();

void Init_LEDs(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();
	
	Enable_Ports();
	
	Init_LEDs();
	
	Init_Ports();
	
	I2C_SetUp();
	
	I2C_WriteRegister(0x69, 0x0F); // Write WHO_AM_I address 

	uint8_t data = I2C_ReadRegister(0x69); // Read from WHO_AM_I register
	
	if (data == 0xD3) // WHO_AM_I register should contain this value
	{
		// SUCCESS - set blue LED HIGH
		GPIOC->ODR |= GPIO_ODR_7; 
	}
	else 
	{
		// FAILURE - set red LED HIGH
		GPIOC->ODR |= GPIO_ODR_6; 
	}
	
	// PART TWO:
	
	//I2C_WriteRegister(0x69, 0x20); // Write CTRL_REG1 address
	// Enable X,Y axes in CTRL_REG1 - enabled by default
	// Set sensor to 'normal mode' using PD bit in CTRL_REG1
	//I2C_WriteRegister(0x69, ); // Set bit 3 to 1 to set to normal mode
	
	// Init Gyro to read X, Y axes

  while (1)
  {
		// Read X and Y axis data registers every 100ms
		// - need to assemble 16-bit measured value from the two data registers for each axis
			
		// Use four LEDs to indicate each measured axis is pos/neg
		// - Set min threshold before changing active LED
		// - Design application such that LED nearest direction of rotation lights up
		HAL_Delay(100);
  }

}

/*

	I2C communication handler for writing to specified device.
	deviceAddr - Address of device communicating on I2C
	data 			 - Either a register address or the data to be written to a register

*/
void I2C_WriteRegister(uint16_t deviceAddr, uint16_t data) 
{
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x1 << 16);
	// Set RD_WRN to WRITE operation - 0 indicates WRITE
	I2C2->CR2 |= (0 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {}
	// Once TXIS flag set continue
	
	// Write data into the TXDR 
	I2C2->TXDR = data;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
}

/*

	I2C communication handler for reading from specified device.
	deviceAddr - Address of device communicating on I2C
	returns		 - 1 byte of data read from specified register
*/
uint8_t I2C_ReadRegister(uint16_t deviceAddr) 
{
	uint8_t data = 0;
	// Use SADD[7:1] bit field in CR2 register to set slave address to 0x69
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x1 << 16);
	// Set RD_WRN to READ operation - 1 indicates READ
	I2C2->CR2 |= (1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
		
	// While RXNE or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {}
	// Once RXNE flag set continue
		
	// Wait for TC flag set
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
		
	// Read contents of RXDR register and return data - remember it is 1 byte at a time
	data = I2C2->RXDR;
		
	// Set STOP bit in CR2 register to release I2C bus
	I2C2->CR2 |= I2C_CR2_STOP;
	
	return data;
}

void I2C_SetUp()
{
	// Enable I2C system clock using RCC register
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Configure bus timing using I2Cx_TIMINGR register to 100kHz standard-mode (taken from figure 5.4)
	I2C2->TIMINGR |= (0x1 << 28); // PRESC
	I2C2->TIMINGR |= (0x13 << 0); // SCLL
	I2C2->TIMINGR |= (0xF << 8); 	// SCHL
	I2C2->TIMINGR |= (0x2 << 16); // SDADEL
	I2C2->TIMINGR |= (0x4 << 20); // SCLDEL

	// Enable I2C peripheral using PE bit in CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
}

void Enable_Ports()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

void Init_Ports() 
{
	// Set PB11 to alt func mode (and PB13)
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13)) 
								| GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1; 
	
	// set PB11 to open-drain output type using OTYPER reg
	GPIOB->OTYPER |= (1 << 11);
	// Select I2C2_SDA as alt func (using AFR register bit pattern: AF1=0001)
	GPIOB->AFR[1] |= 0x1 << GPIO_AFRH_AFSEL11_Pos;
	
	// Set PB13 to alt func mode (done above)
	// set PB13 to open-drain output type
	GPIOB->OTYPER |= (1 << 13);
	// Select I2C2_SCL as alt func (using AFR register bit pattern: AF5=0101)
	GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL13_Pos;
	
	// Set PB14 to output mode, push-pull output type
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1 << 14);
	// set HIGH
	GPIOB->ODR |= GPIO_ODR_14;
	
	// Set PC0 to output mode, push-pull output type
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER &= ~(1 << 0);
	// set HIGH
	GPIOC->ODR |= GPIO_ODR_0;
}

void Init_LEDs(void)
{
	// Set pins to general purpose output mode in MODER register
  GPIOC->MODER |= (1<<12); // PC6 RED
	GPIOC->MODER |= (1<<14); // PC7 BLUE
  GPIOC->MODER |= (1<<16); // PC8 ORANGE
	GPIOC->MODER |= (1<<18); // PC9 GREEN
	
	// Set pins to push-pull output type in OTYPER register
	GPIOC->OTYPER &= ~(1<<6); // PC6
	GPIOC->OTYPER &= ~(1<<7); // PC7
	GPIOC->OTYPER &= ~(1<<8); // PC8
	GPIOC->OTYPER &= ~(1<<9); // PC9
	
	// Set pins to low speed in OSPEEDR register
	GPIOC->OSPEEDR &= ~(1<<12); // PC6
	GPIOC->OSPEEDR &= ~(1<<14); // PC7
	GPIOC->OSPEEDR &= ~(1<<16); // PC8
	GPIOC->OSPEEDR &= ~(1<<18); // PC9
	
	// Set to no pull-up/down resistors in PUPDR register
	GPIOC->PUPDR &= ~((1<<12) | (1<<13)); // PC6
	GPIOC->PUPDR &= ~((1<<14) | (1<<15)); // PC7
	GPIOC->PUPDR &= ~((1<<16) | (1<<17)); // PC8
	GPIOC->PUPDR &= ~((1<<18) | (1<<19)); // PC9
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
