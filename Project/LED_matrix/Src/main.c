/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "font.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* MAX7219 registers */
#define NO_OP_REGISTER 		0x00
#define DIGIT_0_REGISTER 	0x01
#define DIGIT_1_REGISTER 	0x02
#define DIGIT_2_REGISTER 	0x03
#define DIGIT_3_REGISTER 	0x04
#define DIGIT_4_REGISTER 	0x05
#define DIGIT_5_REGISTER 	0x06
#define DIGIT_6_REGISTER 	0x07
#define DIGIT_7_REGISTER 	0x08
#define DECODE_MODE_REGISTER	0x09
#define INTENSITY_REGISTER	0x0A
#define SCAN_LIMIT_REGISTER 	0x0B
#define SHUTDOWN_REGISTER 	0x0C
#define DISPLAY_TEXT_REGISTER	0x0F

/* Shutdown register values */
#define SHUTDOWN_MODE 	0x00
#define NORMAL_OPERATION 	0x01

/* Decode-mode register value */
#define NO_DECODE_ALL_DIGITS 	0x00

/* Intensity register values */
#define INTENSITY_1_32 	0x00
#define INTENSITY_3_32 	0x01
#define INTENSITY_15_32 	0x07
#define INTENSITY_31_32 	0x0F

/* Scan limit register value */
#define DISPLAY_ALL_DIGITS 	0x07

/* Display-test register value */
#define TEST_MODE_OFF 	0x00
#define TEST_MODE_ON 	0x01

int8_t print_char_led(uint8_t character)
{
	  uint8_t data[2] = {};
	  uint8_t i = 0;

	  for(i=0; i<8; i++)
	  {

		HAL_SPI_MspInit(&hspi1);
		data[0] = font[character][7-i];
		data[1] = DIGIT_0_REGISTER + i;
		while(HAL_SPI_Transmit_IT(&hspi1, data, 1) != HAL_OK);
		HAL_SPI_MspDeInit(&hspi1);
	  }

		return 0;

}

int8_t print_string_led(uint8_t* text)
{
	uint32_t i=0;
	for(i=0; i<strlen(text); i++)
	{
		print_char_led(text[i]);
		for(uint32_t j=0; j<2000000; j++);
	}
	return 0;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
    uint8_t data[2] = {};

	HAL_SPI_MspInit(&hspi1);
	data[0] = NORMAL_OPERATION;
	data[1] = SHUTDOWN_REGISTER;
	while(HAL_SPI_Transmit_IT(&hspi1, data, 1) != HAL_OK);
	HAL_SPI_MspDeInit(&hspi1);

	HAL_SPI_MspInit(&hspi1);
	data[0] = NO_DECODE_ALL_DIGITS;
	data[1] = DECODE_MODE_REGISTER;
	while(HAL_SPI_Transmit_IT(&hspi1, data, 1) != HAL_OK);
	HAL_SPI_MspDeInit(&hspi1);

	HAL_SPI_MspInit(&hspi1);
	data[0] = DISPLAY_ALL_DIGITS;
	data[1] = SCAN_LIMIT_REGISTER;
	while(HAL_SPI_Transmit_IT(&hspi1, data, 1) != HAL_OK);
	HAL_SPI_MspDeInit(&hspi1);

	HAL_SPI_MspInit(&hspi1);
	data[0] = INTENSITY_1_32;
	data[1] = INTENSITY_REGISTER;
	while(HAL_SPI_Transmit_IT(&hspi1, data, 1) != HAL_OK);
	HAL_SPI_MspDeInit(&hspi1);

	while(1)
	{
		print_string_led("123456789  ");
		for(uint32_t j=0; j<1000000; j++);
//		for(i=0x20; i<0x7F; i++)
//		{
//			print_char_led(i);
//			for(uint32_t j=0; j<1000000; j++);
//		}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
