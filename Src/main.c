/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */
void CNC_PollSignal_2(void);
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#define PID_PARAM_KP        100            /* Proporcional */
#define PID_PARAM_KI        0.025        /* Integral */
#define PID_PARAM_KD        21            /* Derivative */
#define SERVO_MAX_ERR		40

volatile int32_t Axis_Pos = 0;
volatile int32_t Set_Pos = 0;

volatile int Flag_rotation = 0;
volatile long Aux_Axis_Pos = 0;
float Dac_voltaje = 0.0;
int Flag_Invert = 0;
uint32_t var;
uint16_t OutputScaled = 0;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  int32_t Err_In  = 0;
	  float32_t Out_Response = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  arm_pid_instance_f32 PID;
  PID.Kp = PID_PARAM_KP;        /* Proporcional */
  PID.Ki = PID_PARAM_KI;        /* Integral */
  PID.Kd = PID_PARAM_KD;        /* Derivative */

  arm_pid_init_f32(&PID, 1);

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  TIM3->CCR2 = 0;
  TIM3->CCR1 = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */
	CNC_PollSignal_2();
	if(Axis_Pos >= Set_Pos)
	{
		Err_In = Axis_Pos - Set_Pos;//update error
		Flag_Invert = 0;
	}
	else
	{
		Err_In = Set_Pos - Axis_Pos;//update error
		Flag_Invert = 1;
	}
	if(Err_In >= SERVO_MAX_ERR)
	{
		Out_Response = arm_pid_f32(&PID, Err_In);

		if(Out_Response > 4095) Out_Response = 4095;
		if(Out_Response < 0) Out_Response = 0;

		OutputScaled = Out_Response;

		if(Flag_Invert)
		{
			//Out_Response = 2048 - Out_Response;
			//TIM3->CCR2 = (Out_Response << 4)|(Out_Response >> (28));
			TIM3->CCR2 = OutputScaled << 4;
			TIM3->CCR1 = 0;
		}
		else
		{
			TIM3->CCR2 = 0;
			TIM3->CCR1 = OutputScaled << 4;
			//TIM3->CCR1 = (Out_Response << 4)|(Out_Response >> (28));
			//Out_Response = 2048 + Out_Response;
		}

		//LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, Out_Response);
		//LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
	}
  }
  /* USER CODE END 3 */
}

void CNC_PollSignal_2(void)
{
	//estados anteriores de las terminales del controlador
	static GPIO_PinState CNT_StepPin_Old_State;
	//static GPIO_PinState CNT_DirPin_Old_State;
	//estados anteriores de las terminales del encoder
	static GPIO_PinState ENC_APin_Old_State;
	static GPIO_PinState ENC_BPin_Old_State;
	static GPIO_PinState ENC_RotPin_Old_State;

	//estados actuales de las terminales del controlador
	GPIO_PinState CNT_StepPin_State;
	GPIO_PinState CNT_DirPin_State;
	//estados actuales de las terminales del encoder
	GPIO_PinState ENC_APin_State;
	GPIO_PinState ENC_BPin_State;
	GPIO_PinState ENC_RotPin_State;

	CNT_StepPin_State = LL_GPIO_IsInputPinSet(STEP_GPIO_Port,STEP_Pin);
	CNT_DirPin_State = LL_GPIO_IsInputPinSet(DIR_GPIO_Port,DIR_Pin);
	ENC_APin_State = LL_GPIO_IsInputPinSet(ENC_CHA_GPIO_Port,ENC_CHA_Pin);
	ENC_BPin_State = LL_GPIO_IsInputPinSet(ENC_CHB_GPIO_Port,ENC_CHB_Pin);
	ENC_RotPin_State = LL_GPIO_IsInputPinSet(ENC_CHR_GPIO_Port,ENC_CHR_Pin);

	if(CNT_StepPin_State != CNT_StepPin_Old_State)
	{
		if(CNT_StepPin_State == GPIO_PIN_SET)
		{
			if(CNT_DirPin_State == GPIO_PIN_SET)Set_Pos++;
			else Set_Pos --;
		}
		CNT_StepPin_Old_State = CNT_StepPin_State;
	}

	if(ENC_APin_State != ENC_APin_Old_State)
	{

		if(ENC_APin_State == GPIO_PIN_SET && ENC_BPin_State == GPIO_PIN_SET) Axis_Pos ++;
		if(ENC_APin_State == GPIO_PIN_RESET && ENC_BPin_State == GPIO_PIN_SET) Axis_Pos --;
		if(ENC_APin_State == GPIO_PIN_SET && ENC_BPin_State == GPIO_PIN_RESET) Axis_Pos --;
		if(ENC_APin_State == GPIO_PIN_RESET && ENC_BPin_State == GPIO_PIN_RESET) Axis_Pos ++;

		ENC_APin_Old_State = ENC_APin_State;
	}

	if(ENC_BPin_State != ENC_BPin_Old_State)
	{
		if(ENC_APin_State == GPIO_PIN_SET && ENC_BPin_State == GPIO_PIN_SET) Axis_Pos --;
		if(ENC_APin_State == GPIO_PIN_RESET && ENC_BPin_State == GPIO_PIN_SET) Axis_Pos ++;
		if(ENC_APin_State == GPIO_PIN_SET && ENC_BPin_State == GPIO_PIN_RESET) Axis_Pos ++;
		if(ENC_APin_State == GPIO_PIN_RESET && ENC_BPin_State == GPIO_PIN_RESET) Axis_Pos --;
		ENC_BPin_Old_State = ENC_BPin_State;
	}
	//Por implementar
	if(ENC_RotPin_State != ENC_RotPin_Old_State)
	{
		if(ENC_RotPin_State == GPIO_PIN_SET)
		{
			Aux_Axis_Pos = Axis_Pos;
			Axis_Pos = 0;
			Flag_rotation = 1;
		}
		//Sucede cuando el motor da una rotacion
		ENC_RotPin_Old_State = ENC_RotPin_State;
	}
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	//HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)== 0
	//HAL_
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
