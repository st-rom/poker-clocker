/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "gpio.h"




/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

GPIO_InitTypeDef _GPIO_InitStructKeypad;

#define RCC_APB2Periph_GPIOA             ((uint32_t)0x00000004)

#define KEYPAD_RCC_GPIO_COL		RCC_APB2Periph_GPIOA
#define KEYPAD_GPIO_COL0				GPIOA
#define KEYPAD_GPIO_COL1			GPIOC
#define KEYPAD_GPIO_COL2				GPIOB
#define KEYPAD_GPIO_COL3				GPIOE
#define KEYPAD_PIN_COL0				GPIO_PIN_7
#define KEYPAD_PIN_COL1				GPIO_PIN_5
#define KEYPAD_PIN_COL2				GPIO_PIN_1
#define KEYPAD_PIN_COL3				GPIO_PIN_7
// GPIO pin definitions for keypad rows (must on the same GPIO)
#define KEYPAD_RCC_GPIO_ROW		RCC_APB2Periph_GPIOA
#define KEYPAD_GPIO_ROW				GPIOA
#define KEYPAD_GPIO_ROW1				GPIOF
#define KEYPAD_PIN_ROW0				GPIO_PIN_1
#define KEYPAD_PIN_ROW1				GPIO_PIN_3
#define KEYPAD_PIN_ROW2				GPIO_PIN_4
#define KEYPAD_PIN_ROW3				GPIO_PIN_5
// Return value for no key pressed
#define KEYPAD_NO_PRESSED			0xFF
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void KeypadInit(void);
uint8_t KeypadGetKey(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
void KeypadInit()
{

	DelayInit();

	// GPIO clock for keypad columns and rows
	RCC_APB2PeriphClockCmd(GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(GPIOE, ENABLE);

	RCC_APB2PeriphClockCmd(GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(GPIOF, ENABLE);

	// Configure GPIO as output open drain for keypad columns

	_GPIO_InitStructKeypad.GPIO_Pin = KEYPAD_PIN_COL0 | KEYPAD_PIN_COL1 |
		KEYPAD_PIN_COL2 | KEYPAD_PIN_COL3;
	_GPIO_InitStructKeypad.GPIO_Mode = GPIO_Mode_Out_OD;
	_GPIO_InitStructKeypad.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(KEYPAD_GPIO_COL, &_GPIO_InitStructKeypad);

	// Configure GPIO as input with pull-up resistor for keypad rows
	_GPIO_InitStructKeypad.GPIO_Pin = KEYPAD_PIN_ROW0 | KEYPAD_PIN_ROW1 |
		KEYPAD_PIN_ROW2 | KEYPAD_PIN_ROW3;
	_GPIO_InitStructKeypad.GPIO_Mode = GPIO_Mode_IPU;
	_GPIO_InitStructKeypad.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(KEYPAD_GPIO_ROW, &_GPIO_InitStructKeypad);

}
*/


uint8_t KeypadGetKey()
{
	const unsigned gpios[] = {KEYPAD_GPIO_COL0, KEYPAD_GPIO_COL1, KEYPAD_GPIO_COL2, KEYPAD_GPIO_COL3};
	const unsigned pins[] = {KEYPAD_PIN_COL0, KEYPAD_PIN_COL1, KEYPAD_PIN_COL2, KEYPAD_PIN_COL3};
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			if(i == j){
				HAL_GPIO_WritePin(gpios[i], pins[i], GPIO_PIN_SET);
			}
			else{
				HAL_GPIO_WritePin(gpios[i], pins[i], GPIO_PIN_RESET);
			}
		}
		if(HAL_GPIO_ReadPin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0)){
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
				return '1';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
				return '4';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW2))
				return '7';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
				return '*';
		}
		if(HAL_GPIO_ReadPin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1)){
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
				return '2';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
				return '5';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW2))
				return '8';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
				return '0';
		}
		if(HAL_GPIO_ReadPin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2)){
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
				return '3';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
				return '6';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW2))
				return '9';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
				return '#';
		}
		if(HAL_GPIO_ReadPin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3)){
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
				return 'A';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
				return 'B';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW2))
				return 'C';
			if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
				return 'D';
		}
		return KEYPAD_NO_PRESSED;
	}
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	 extern void initialise_monitor_handles(void);
	 initialise_monitor_handles();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t key;
  while (1)
  {
	  	  	key = KeypadGetKey();

	  		// Display pressed char to LCD
	  		if (key != KEYPAD_NO_PRESSED)
	  		{
	  			printf("%c\n", key);
	  		}
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
