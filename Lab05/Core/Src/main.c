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

void init_leds(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();
	
	init_leds();
	
	// 5.1 Wire discovery board so it uses I2C instead of SPI - see lab manual
	
	
	// 5.2:
	// Enable GPIOB and GPIOC in RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// Set PB11 to alt func mode (and PB13)
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13)) 
								| GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1; 
	
	// set PB11 to open-drain output type using OTYPER reg
	GPIOB->OTYPER |= (1 << 11);
	// Select I2C2_SDA as alt func (using AFR register)
	
	// Set PB13 to alt func mode (done above)
	// set PB13 to open-drain output type
	GPIOB->OTYPER |= (1 << 13);
	// Select I2C2_SCL as alt func (using AFR register)
	
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
	
	// 5.3: I2C set-up
	// Enable I2C system clock using RCC register
	// Configure bus timing using I2Cx_TIMINGR register to 100kHz standard-mode
	// Enable I2C peripheral using PE bit in CR1 register
	
	// 5.4: Transaction set-up
	// Use SADD[7:1] bit field in CR2 register to set slave address to 0x6B
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	// Set RD_WRN to Write operation
	// Set START bit to begin the address frame
	
	// While TXIS or NACKF flags not set wait
	// Once TXIS flag set continue
	// Write address of WHO_AM_I register into the TXDR 
	// Wait until TC flag set
	
	// Load same parameters as above but now with RD_WRN set to read operaiton
	// While RXNE or NACKF flags not set wait
	// Once RXNE flag set continue
	// Wait for TC flag set
	// Check contents of RXDR register to see if it matches 0xD4
	// If it matches toggle an LED to indicate success
	// Set STOP bit in CR2 register to release I2C bus
	

  while (1)
  {
		
  }

}

void init_leds(void)
{
	// Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set pins PC6 to general purpose output mode in MODER register
  GPIOC->MODER |= (1<<12); // PC6
	GPIOC->MODER |= (1<<14); // PC7
  GPIOC->MODER |= (1<<16); // PC8
	GPIOC->MODER |= (1<<18); // PC9
	
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
	
	// Set green PC9 high
	GPIOC->ODR |= GPIO_ODR_9; // PC9 (green) high
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
