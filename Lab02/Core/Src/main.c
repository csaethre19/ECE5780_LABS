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

void EXTI0_1_IRQHandler(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// Init all LEDs:
	
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
	
	// Setting up USER push-button:
	
	// Enable the GPIOA clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Set pins for push-button to input mode in MODER register
	GPIOA->MODER &= ~(3);
	
	// Set pins to low speed in OSPEEDR register
	GPIOA->OSPEEDR &= ~(3);
	
	// Enable pull-down resistor in PUPDR register
	GPIOA->PUPDR |= (2);	
	
  /* Configure the system clock */
  SystemClock_Config();
	
	// Set input line 0 (EXTI0) for EXTI peripheral to generate interrupts on rising-edge of user button:
	
	// Setting mask (enabled/unmask) bit in EXTI_IMR register
	EXTI->IMR = 0x0001;
	// Setting EXTI input line 0 to have rising-edge trigger
	EXTI->RTSR = 0x0001;
	
	// Use RCC to enable the peripheral clock to the SYSCFG peripheral
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// Configure multiplexer to route PA0 (user button) to the EXTI input line 0 (EXTI0)
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	// Enable EXTI0_1_IRQn for EXTI line 0 interrupt by passing defined name to NVIC_EnableIRQ()
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	// Setting interrupt to 1 (high-priority)
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	
  while (1)
  {
		// Toggle red PC6 with moderate-slow delay (400-600ms) 
		// Use this LED to determine if system is stuck in interrupt
		HAL_Delay(450); // Delay 450ms
		GPIOC->ODR ^= GPIO_ODR_6;
		
  }
}

void EXTI0_1_IRQHandler(void) {
	// Toggle green and orange LEDs
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	// Clear flag for input line 0 in the EXTI pending register
	EXTI->PR |= EXTI_PR_PR0;
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
