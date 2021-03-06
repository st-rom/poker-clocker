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


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define KEYPAD_PIN_COL0				GPIO_PIN_7
#define KEYPAD_PIN_COL1				GPIO_PIN_5
#define KEYPAD_PIN_COL2				GPIO_PIN_1
#define KEYPAD_PIN_COL3				GPIO_PIN_7
#define KEYPAD_GPIO_COL0			GPIOA
#define KEYPAD_GPIO_COL1			GPIOC
#define KEYPAD_GPIO_COL2			GPIOB
#define KEYPAD_GPIO_COL3			GPIOE
const unsigned gpios[] = {KEYPAD_GPIO_COL0, KEYPAD_GPIO_COL1, KEYPAD_GPIO_COL2, KEYPAD_GPIO_COL3};
const unsigned pins[] = {KEYPAD_PIN_COL0, KEYPAD_PIN_COL1, KEYPAD_PIN_COL2, KEYPAD_PIN_COL3};
// Return value for no key pressed

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t KeypadGetKey(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int my_time[7] = {0, 0, 0, 0, 0, 0 ,0};
int kom = 0;
int current = 0;
int dec = 0;
double start = 0;
double changed = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 printf("%d\n", gpios[3] == 1);
 if( GPIO_Pin == UButton0_Pin || GPIO_Pin == UButton1_Pin || GPIO_Pin == UButton2_Pin || GPIO_Pin == UButton3_Pin)
 {
  static uint32_t last_change_tick;
  if( HAL_GetTick() - last_change_tick < 20 )
  {
   return;
  }
  last_change_tick = HAL_GetTick();
  int length = strlen(my_time);
  const unsigned ubuttons[] = {UButton0_Pin, UButton1_Pin, UButton2_Pin, UButton3_Pin};
  char symbs[4][4] = {{'1', '4', '7', '*'}, {'2', '5', '8', '0'}, {'3', '6', '9', '#'}, {'A', 'B', 'C', 'D'}};
  for(int i = 0; i < 4; i++){
	  if(HAL_GPIO_ReadPin(gpios[i], pins[i])){
		  for(int j = 0; j < 4; j++){
			  //printf("%d, %d,\n", GPIO_Pin == UButton2_Pin, i);
			  if (GPIO_Pin == ubuttons[j]){
				  printf("%c %d %d\n", symbs[i][j], GPIO_Pin == UButton2_Pin, i == 3);
			  }
			  if (GPIO_Pin == UButton2_Pin && i == 3 && length != 0 && start == 0){
				  kom = 0;
				  dec = 0;
				  return;
			  }
			  else if(GPIO_Pin == UButton1_Pin && i == 3 && current < 7 && start == 0){
				  my_time[current] = kom;
				  current++;
				  kom = 0;
				  dec = 0;
				  changed = 1;
				  return;
			  }
			  else if(GPIO_Pin == UButton0_Pin && i == 3 && current > 0 && start == 0){
				  my_time[current] = kom;
				  current--;
				  dec = 2;
				  changed = 1;
				  return;
			  }
			  else if(GPIO_Pin == UButton3_Pin && i == 3){
				  if (start == 0){
					  start = 1;
				  }
				  else{
					  start = 0;
				  }
				  changed = 1;
				  return;
			  }
			  else if (GPIO_Pin == ubuttons[j] && symbs[i][j] != '*' && symbs[i][j] != '#' && i != 3 && start == 0){
				  if (dec == 0){
					  kom += symbs[i][j] - '0';
				  }
				  else if (dec == 1){
					  kom *= 10;
					  kom += (symbs[i][j] - '0');
				  }
				  dec++;
				  return;
			  }
		  }
	  }
  }
 }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


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
  int i = 0;
  while (1)
  {
	 if(changed == 1){
		 __disable_irq();
		 /*
		 for(int i = 0; i < strlen(my_time); i++){
			 if(i % 2 == 0){
				 printf(" ");
			 }
			 printf("%c", my_time[i]);
		 }
		 */
		 printf("year -- %d, month -- %d, date -- %d, weekday -- %d\n%d:%d:%d\n", my_time[0], my_time[1], my_time[2], my_time[3], my_time[4], my_time[5], my_time[6]);
		 changed = 0;
		 __enable_irq();
	 }
	 //for(int i = 0; i < 4; i++){
		 __disable_irq();
		 HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		 for(int j = 0; j < 4; j++){
			 if(i == j){
				 HAL_GPIO_WritePin(gpios[j], pins[j], GPIO_PIN_SET);
			 }
			 else{
				 HAL_GPIO_WritePin(gpios[j], pins[j], GPIO_PIN_RESET);
			 }
		 }
		 __enable_irq();
		 i++;
		 if(i == 4){
			 i = 0;
		 }
	 //}
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
