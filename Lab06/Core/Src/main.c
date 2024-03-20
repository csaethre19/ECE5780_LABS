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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();
	
	Init_LEDs();
	
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

	USART_SetUp();
	
		
  while (1)
  {
		// Read ADC data register 
		int16_t data = ADC1->DR;
		// Turn on/off LEDs depending on the value
		USART_Transmit_String("Data: ");
		USART_Transmit_Number(data);
		USART_Transmit_Newline();
	
		// Use four increasing threshold values, each LED should have a min ADC value to turn on 
		// voltage increases -> LEDs should light one by one
		// if pin voltage decreases below threshold for an LED -> turn off
		
		// LED    -> THRESHOLD:
		// RED    -> 20
		// Orange -> 80
		// Green  -> 140
		// Blue   -> 200
		if (data >= 20) GPIOC->ODR |= GPIO_ODR_6;
		else GPIOC->ODR &= ~GPIO_ODR_6;
		if (data >= 80) GPIOC->ODR |= GPIO_ODR_8;
		else GPIOC->ODR &= ~GPIO_ODR_8;
		if (data >= 140) GPIOC->ODR |= GPIO_ODR_9;
		else GPIOC->ODR &= ~GPIO_ODR_9;
		if (data >= 200) GPIOC->ODR |= GPIO_ODR_7;
		else GPIOC->ODR &= ~GPIO_ODR_7;

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

void USART_SetUp() {
	// Configure pins PC4 and PC5 to alternate function mode:
	// PC4 -> USART_3TX (transmitter)
	// PC5 -> USART_3RX (receiver)
	// Use bit pattern for AF1 -> 0001
	// Using GPIOC_AFRL register
	// BAUD RATE: 115200
	
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
}

void USART_Transmit_Byte(uint8_t b)
{
	while (!(USART3->ISR & (1 << 7))) 
	{
		// Wait for data to be transferred to shift register - transmit register is empty
	}
	
	// Write the byte into the transmit data register
	USART3->TDR = b;
}

void USART_Transmit_String(const char* str)
{
    while (*str)
    {
        USART_Transmit_Byte((uint8_t)(*str));
        str++;
    }
}

void USART_Transmit_Number(int16_t number)
{
		int base = 10;
    char numberString[7]; // "-32768" + null terminator
    char* ptr = numberString, *ptr1 = numberString, tmp_char;
    int tmp_value;

    do {
        tmp_value = number;
        number /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - number * base)];
    } while ( number );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
		
		USART_Transmit_String(numberString);
}

void USART_Transmit_Newline()
{
    USART_Transmit_Byte('\r'); // Carriage Return
    USART_Transmit_Byte('\n'); // Line Feed
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
