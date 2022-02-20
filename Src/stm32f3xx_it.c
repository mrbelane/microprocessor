/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include <LiquidCrystal.h>
#include <stdbool.h>
#include <string.h>

uint16_t pinsLed[3] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10};


bool flagFirstTemprature = true;
bool isSafe = true; // 1:is safe 0:is not safe(fire)

const int MAX_DATA_LINE_SIZE = 4;
int currentTemperature = 0;
int valuePir = 0;
int changePir = 0;
int setSegment = 0;


unsigned char dataLine[MAX_DATA_LINE_SIZE];
extern unsigned char data;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

extern int counter;
extern int autoSelected;
extern int temperature;
extern int TEMPERATURE_LIGTH_SCREEN;
extern int screen;
extern bool start;
extern int light;
extern int volume;
extern int PIR;
extern int mode;
extern int ligthValue;
extern int volumeValue;
extern uint16_t pins[8];
extern int firstTempture;
extern int  saftyMode;


// ---- FUNCTIONS -----

void sevenSegF_Safe(){
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); // enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0); // b
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0); // c
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0); // d
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
}

void sevenSegF_Fire(){
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); // enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0); // b
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0); // c
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0); // d
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
}
void sevenSegI(){
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); // enable
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0); // a
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0); // b
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0); // c
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0); // d
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // g
	
	
}
void sevenSegA(){
		// is safe
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1	, 0); // enable
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // a
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); // a
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1); // b
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1); // c
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0); // d
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
	
}

void sevenSegR(){

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2	, 0); // enable
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // a
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); // a
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a
	
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1); // b
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1); // c
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0); // d
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
}

void sevenSegE(){
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0); // enable

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0); // b
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0); // c
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1); // d
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1); // e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); // a
}

void sevenSegS(){
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); // enable

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1); // a
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0); // b
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1); // c
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1); // d
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0); // e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // f
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // g
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1); // a	
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

		

//	
	
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
	
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 and Touch Sense controller.
*/
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */

  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	
		light = HAL_ADC_GetValue(&hadc2);
		
		if(screen == TEMPERATURE_LIGTH_SCREEN){	
			ligthValue = ((int)light * 100 / 4095);
			char buffer[10];

			sprintf(buffer,"%d ", ligthValue);

			setCursor(3, 2);
			print(buffer);
			
			HAL_ADC_Start_IT(&hadc2);
		}
		
		if(mode == 0){
			light = HAL_ADC_GetValue(&hadc2);
			ligthValue = ((int)light * 100 / 4095);
				
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ligthValue);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ligthValue);
				HAL_ADC_Start_IT(&hadc2);
			
		}else if(mode == 1){
			
					volume = HAL_ADC_GetValue(&hadc1);
					volumeValue = (volume*16/60)+1;
					if(volumeValue > 60){
						volumeValue = 60;
					}
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, volumeValue);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, volumeValue);
			
				HAL_ADC_Start_IT(&hadc1);
		}
		else if(mode == 2){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		}
	
		
	
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
		if(mode == 0 && autoSelected){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ligthValue);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ligthValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ligthValue);
		}
		else if(mode == 1 && autoSelected){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, volumeValue);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, volumeValue);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, volumeValue);
		}
		if(!isSafe){
			for(int i = 0; i < 3; i++){
				HAL_GPIO_TogglePin(GPIOA, pinsLed[i]);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			}
		}
	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
	if(isSafe){
		// safe
		switch(setSegment % 4){
			
			case 0:
				sevenSegS();
				setSegment++;
			break;
			case 1:
				sevenSegA();
				setSegment++;
				break;
			case 2:
				sevenSegF_Safe();
				setSegment++;
				break;
			case 3: 
				sevenSegE();
				setSegment++;
				break;
		}
	}
	else if(!isSafe){
		// fire
		switch(setSegment % 4){
			
			case 0:
				sevenSegF_Fire();
				setSegment++;
			break;
			case 1:
				sevenSegI();
				setSegment++;
				break;
			case 2:
				sevenSegR();
				setSegment++;
				break;
			case 3: 
				sevenSegE();
				setSegment++;
				break;
		}	
	}
	
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	if(mode == 0){
		
	}
	if(data == '\r'){
		if(dataLine[0] == 'S' && dataLine[1] == 'A' && dataLine[2] == 'F' && dataLine[3] == 'E' && counter < 5){
			isSafe = true;
		}
		for(int i = 0; i < MAX_DATA_LINE_SIZE;i++){
			dataLine[i] = '\0';
		}
		counter = 0;
	}else if(counter < MAX_DATA_LINE_SIZE){
		
		dataLine[counter] = data;
		counter++;
	}
	HAL_UART_Receive_IT(&huart2, &data, sizeof(data));
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles ADC3 global interrupt.
*/
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */

		if(flagFirstTemprature){
			firstTempture = HAL_ADC_GetValue(&hadc3);
			firstTempture = (firstTempture  / 10);
			flagFirstTemprature = false;
		}
		temperature = HAL_ADC_GetValue(&hadc3);
	
		if(screen == TEMPERATURE_LIGTH_SCREEN){
			
				currentTemperature = (temperature  / 10);
			
//				if(firstTempture <= currentTemperature + 4){
			if(currentTemperature>  35){
					isSafe =  false;
				}
				else{
					isSafe = true;
				}
				
				char buffer[10];
				sprintf(buffer,"%d ", currentTemperature);

				setCursor(2, 1);
				print(buffer);
		}
		HAL_ADC_Start_IT(&hadc3);
  /* USER CODE END ADC3_IRQn 1 */
}

/**
* @brief This function handles ADC4 interrupt.
*/
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */

  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */

		PIR = HAL_ADC_GetValue(&hadc4);

		valuePir = PIR / 4095;
			if(valuePir == 1){
				HAL_TIM_Base_Start(&htim2);	
			}	
			
		 else {
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 1);
		 }	 
 
	HAL_ADC_Start_IT(&hadc4);

  /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
