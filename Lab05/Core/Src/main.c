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
#include <stdlib.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void Init_LEDs(void);

void Enable_GPIO_Clks();

void I2C_Ports_Config();

void I2C_SetUp();

void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data);

int8_t I2C_ReadRegister(uint16_t deviceAddr);

void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr);

void USART_SetUp();

void USART_USART_Transmit_Byte(uint8_t b);

void USART_Transmit_String(const char* str);

void USART_Transmit_Number(int16_t number);

void USART_Transmit_Newline();

#define L3GD20 0x69

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Configure the system clock */
  SystemClock_Config();
	
	Enable_GPIO_Clks();
	
	Init_LEDs();
	
	I2C_Ports_Config();
	
	I2C_SetUp();
	
	// PART ONE:
	//I2C_WriteRegister(L3GD20, 0x0F); // Write WHO_AM_I address 

	//uint8_t data = I2C_ReadRegister(L3GD20); // Read from WHO_AM_I register
	
	//if (data == 0xD3) 	GPIOC->ODR |= GPIO_ODR_7; // SUCCESS - set blue LED HIGH
	//else GPIOC->ODR |= GPIO_ODR_6; // FAILURE - set red LED HIGH
	
	// Set up USART for debugging
	USART_SetUp();
	
	// PART TWO:
	
	// Init Gyro to read X, Y axes
	// Enable X,Y axes in CTRL_REG1 - enabled by default
	// Set sensor to 'normal mode' using PD bit in CTRL_REG1 - bit 3 to 1 for normal mode
	
	int8_t normal_mode = 0xB;
	int8_t ctrl_reg1_addr = 0x20;
	I2C_WriteRegister(L3GD20, ctrl_reg1_addr, normal_mode); 

  while (1)
  {
		
		// Read X and Y axis data registers every 100ms:
		// - need to assemble 16-bit measured value from the two data registers for each axis
		
		// Use four LEDs to indicate each measured axis is pos/neg
		// - Set min threshold before changing active LED
		// - Design application such that LED nearest direction of rotation lights up
		
		I2C_SetRegAddress(L3GD20, 0x28); 
		int8_t x_low = I2C_ReadRegister(L3GD20); 
		
		char xl_str[] = "x_low: ";
		USART_Transmit_String(xl_str);
		USART_Transmit_Number(x_low);
		USART_Transmit_Newline();
		
		I2C_SetRegAddress(L3GD20, 0x29);
		int8_t x_high = I2C_ReadRegister(L3GD20); 
		
		char xh_str[] = "x_high: ";
		USART_Transmit_String(xh_str);
		USART_Transmit_Number(x_high);
		USART_Transmit_Newline();
		
		int16_t x_data = ((int16_t)x_high << 8) | (uint8_t)x_low;
	
		char xd_str[] = "X: ";
		USART_Transmit_String(xd_str);
		USART_Transmit_Number(x_data);
		USART_Transmit_Newline();
		
		I2C_SetRegAddress(L3GD20, 0x2A); 
		int8_t y_low = I2C_ReadRegister(L3GD20); 
				
		char yl_str[] = "y_low: ";
		USART_Transmit_String(yl_str);
		USART_Transmit_Number(y_low);
		USART_Transmit_Newline();
		
		I2C_SetRegAddress(L3GD20, 0x2B);
		int8_t y_high = I2C_ReadRegister(L3GD20);
		
		char yh_str[] = "y_high: ";
		USART_Transmit_String(yh_str);
		USART_Transmit_Number(y_high);
		USART_Transmit_Newline();
		
		int16_t y_data = ((int16_t)y_high << 8) | (uint8_t)y_low;
		
		char y_str[] = "Y: ";
		USART_Transmit_String(y_str);
		USART_Transmit_Number(y_data);
		USART_Transmit_Newline();
	
		int32_t threshold = 1000;
		
		GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_8 | GPIO_ODR_9); // Reset the ODR bits for LEDs

		if (y_data > threshold) {
				GPIOC->ODR |= GPIO_ODR_6; // Red LED for positive Y 
		} else if (y_data < -threshold) {
				GPIOC->ODR |= GPIO_ODR_7; // Blue LED for negative Y 
		}

		if (x_data > threshold) {
				GPIOC->ODR |= GPIO_ODR_9; // Green LED for positive X
		} else if (x_data < -threshold) {
				GPIOC->ODR |= GPIO_ODR_8; // Orange LED for negative X
		}
		
		HAL_Delay(1000);

  }

}

void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr)
{
	I2C2->CR2 = 0; // clear register
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x1 << 16);
	// Set RD_WRN to WRITE operation - 0 indicates WRITE
	I2C2->CR2 &= ~(1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {} // getting stuck here on second call to this function!
	// Once TXIS flag set continue
		
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
	}
	
	// Write data into the TXDR 
	I2C2->TXDR = regAddr;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
}

/*

	I2C communication handler for writing to specified device.
	deviceAddr - Address of device communicating on I2C
	data 			 - Either a register address or the data to be written to a register

*/
void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data) 
{
	I2C2->CR2 = 0; // clear register
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x2 << 16);
	// Set RD_WRN to WRITE operation - 0 indicates WRITE
	I2C2->CR2 &= ~(1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {} // getting stuck here on second call to this function!
	// Once TXIS flag set continue
		
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
	}
	
	// Set reg address
	I2C2->TXDR = regAddr;

	
	while (!(I2C2->ISR & I2C_ISR_TXIS)) {}
		
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
int8_t I2C_ReadRegister(uint16_t deviceAddr) 
{
	I2C2->CR2 = 0; // clear register
	int8_t data = 0;

	// Use SADD[7:1] bit field in CR2 register to set slave address to L3GD20
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
	
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		GPIOC->ODR |= GPIO_ODR_8; // ORANGE - I2C not working!
	}
		
	// Wait for TC flag set
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
		
	// Read contents of RXDR register and return data - remember it is 1 byte at a time
	data = I2C2->RXDR;
	
	return data;
}

void I2C_SetUp()
{
	// PB11 -> SDA Line (data)
	// PB13 -> SCL Line (clock)
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

void USART_SetUp()
{
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

void Enable_GPIO_Clks()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

void I2C_Ports_Config() 
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
