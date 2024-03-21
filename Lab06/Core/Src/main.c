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

#include "main.h"


void SystemClock_Config(void);

void Init_LEDs(void);

void USART_SetUp();

void USART_USART_Transmit_Byte(uint8_t b);

void USART_Transmit_String(const char* str);

void USART_Transmit_Number(int16_t number);

void USART_Transmit_Newline();

void ADC_Threshold_LEDs();

void ADC_Config();

void DAC_Config();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();
	
	Init_LEDs();
	
	ADC_Config();
	
	DAC_Config();
	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

	uint16_t index;
		
  while (1)
  {
		HAL_Delay(1);
		
		DAC->DHR8R1 = sine_table[index];
		
		if (index == 31) index = 0;
		else             index++;
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

void ADC_Config()
{
// PC0 -> ADC_IN10
	// set PC0 to analog mode and no pull-up/down resistors
	GPIOC->MODER |=  (1<<0); 
	GPIOC->MODER |=  (1<<1); 
	GPIOC->PUPDR &= ~(1<<0);
	GPIOC->PUPDR &= ~(1<<1);
	
	// Connect center pin of pot to PC0 and legs to 3V and GND
	
	// Enable the ADC1 in the RCC 
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
	// Configure ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled
	ADC1->CFGR1 |=  (1<<4);     // 8-bit resolution - bits [4:3] b10
	ADC1->CFGR1 &=  ~(1<<3);
	ADC1->CFGR1 |=  (1<<13);     // continuous conversion mode - bit 13 b01
	ADC1->CFGR1 &= ~(1<<11);    // hardware triggers disabled - bits [11:10] b00
	ADC1->CFGR1 &= ~(1<<10);
	
	// Select/enable the input pin's channel for ADC conversion:
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	
	// Perform self-calibration, enable, and start ADC:
	
	/* From Code Example in Datasheet */
	if ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		ADC1->CR |= ADC_CR_ADDIS; // clear ADEN by setting ADDIS
	}
	
	while ((ADC1->CR & ADC_CR_ADEN) != 0) {} 
		
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // clear DMAEN
	ADC1->CR |= ADC_CR_ADCAL; // Launch calibration by setting ADCAL
		
	while ((ADC1->CR & ADC_CR_ADCAL) !=0) {} // Wait until ADCAL=0
	
	/* From Code Example in Datasheet */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
	{
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	
	ADC1->CR |= ADC_CR_ADEN; // Enabling ADC
	
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) 
	{
		// wait until ADC Ready
	}
	
	// Start ADC	
	ADC1->CR |= ADC_CR_ADSTART;
	
}

void DAC_Config() 
{
	// Configure DAC GPIO pin to analog mode / no pull/push resistors (PA4 -> DAC_OUT1 channel 1)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	GPIOA->MODER |=  (1<<9); 
	GPIOA->MODER |=  (1<<8); 
	GPIOA->PUPDR &= ~(1<<9);
	GPIOA->PUPDR &= ~(1<<8);
	
	// Connect oscilloscope to pin
	
	// Set the used DAC channel to software trigger mode
	DAC->CR |= (1<<5);
	DAC->CR |= (1<<4);
	DAC->CR |= (1<<3);
	
	// Enable used DAC channel
	DAC->CR |= (1<<0);
}

void ADC_Threshold_LEDs()
{
		// LED    -> THRESHOLD:
		// RED    -> 20
		// Orange -> 80
		// Green  -> 140
		// Blue   -> 200
	
		int16_t data = ADC1->DR;
	
		if (data >= 20) GPIOC->ODR |= GPIO_ODR_6;
		else GPIOC->ODR &= ~GPIO_ODR_6;
		if (data >= 80) GPIOC->ODR |= GPIO_ODR_8;
		else GPIOC->ODR &= ~GPIO_ODR_8;
		if (data >= 140) GPIOC->ODR |= GPIO_ODR_9;
		else GPIOC->ODR &= ~GPIO_ODR_9;
		if (data >= 200) GPIOC->ODR |= GPIO_ODR_7;
		else GPIOC->ODR &= ~GPIO_ODR_7;
}

void Init_LEDs(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
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
