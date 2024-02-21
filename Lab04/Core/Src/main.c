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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void transmit_char(char ch);

void transmit_string(char chArray[]);

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
	
	// Configure pins PC4 and PC5 to alternate function mode:
	// PC4 -> USART_3TX (transmitter)
	// PC5 -> USART_3RX (receiver)
	// Use bit pattern for AF1 -> 0001
	// Using GPIOC_AFRL register
	
	GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5)) 
								| GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; 
								
	// Select appropriate function number in alternate function registers 
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL4_Pos;
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL5_Pos;
	
	// Enable system clock for USART3 in RCC peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set Baud rate for communication to 115200 bits/second
	uint32_t clk_freq = HAL_RCC_GetHCLKFreq();
	USART3->BRR = clk_freq / 115200;
	
	// Enable transmitter and receiver hardware
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable USART3
	USART3->CR1 |= USART_CR1_UE;

  while (1)
  {
		//transmit_char('c');
		//HAL_Delay(200); // Delay 200ms		
		
		// TODO1: once transmit_string() complete - modify to transmit a short phrase instead of a single character
		
		// TODO2: check the USART status flag is not empty
		// 			 use switch statement to check received char is 'r', 'b', 'o', 'g' for red, blue, orange, green 
		//		   switch the corresponding LED when matched
		//       default case: print error message to console (use transmit_string()?)
		//       Receive data register: USART3->RDR 
  }

}

void transmit_char(char ch) 
{
	while (!(USART3->ISR & (1 << 7))) 
	{
		// Wait for data to be transferred to shift register - transmit register is empty
	}
	
	// Write the character into the transmit data register
	USART3->TDR = ch;
}

void transmit_string(char chArray[]) 
{
	uint32_t i = 0;
	while (chArray[i] != 0)
	{
		transmit_char(chArray[i]);
		i++;
	}
}

void init_leds(void)
{
	// Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set pins PC6 to general purpose output mode in MODER register
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
