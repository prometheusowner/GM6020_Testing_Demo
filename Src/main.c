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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "pid.h"
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
extern moto_info_t motor_info[MOTOR_MAX_NUM];
int16_t led_cnt;
pid_struct_t motor_pid[7];
float target_speed;
uint16_t t_flag;
uint16_t target_angle;
uint16_t pwm_pulse = 1080;  // default pwm pulse width:1080~1920
uint16_t direction = 0;		// default direction is 0 or 1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET); // switch on 24v power
  pwm_init();                              // start pwm output
  can_user_init(&hcan1);                   // config can filter, start can
	t_flag = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    pid_init(&motor_pid[i], 40, 3, 0, 30000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	target_speed = 0.0f;
	t_flag = 1;
	for (uint8_t i = 0; i < 7; i++)
    {
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed, motor_info[i].rotor_speed);
    }
    /* send motor control message through can bus*/
    set_motor_voltage(0, 
                      motor_info[0].set_voltage, 
                      motor_info[1].set_voltage, 
                      motor_info[2].set_voltage, 
                      motor_info[3].set_voltage);
    
    set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
    HAL_Delay(3);
		HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
		HAL_Delay(3);
		
  while (1)
  {
    /* led blink */
    led_cnt ++;
    if (led_cnt == 250)
    {
      led_cnt = 0;
      LED_RED_TOGGLE(); //blink cycle 500ms
    }
    
    /* scan is key be pressd down to change target speed*/
    /* direction control with the speed closed-loop*/
		/*if (key_scan())
    {	
			direction = ~direction;
			if(direction)
			{
				  target_speed = 100.0f;  // target speed increase 100rpm in ccw
			}
			else
			{
				  target_speed = -100.0f;  // target speed increase 100rpm in cw
			}
    }*/
		
		/* reset to the initial position*/
		
    if (key_scan())
		{
			if (t_flag == 0)
			{
				if ((motor_info[0].rotor_angle >= 4000) && (motor_info[0].rotor_angle >= 8000))
				{
					target_angle = 2000;
				}
				else
				{
					target_angle = 6000;		
				}
			}
			else
			{
				t_flag = 0;
			}
		}
		

    /* motor speed pid calc */
    for (uint8_t i = 0; i < 7; i++)
    {
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed, motor_info[i].rotor_speed);
    }
    /* send motor control message through can bus*/
    set_motor_voltage(0, 
                      motor_info[0].set_voltage, 
                      motor_info[1].set_voltage, 
                      motor_info[2].set_voltage, 
                      motor_info[3].set_voltage);
    
    set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
    HAL_Delay(3);
		HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
		if ((motor_info[0].rotor_angle - target_angle)<3 && (motor_info[0].rotor_angle - target_angle)>-3)
		{
			target_speed = 0.0f;
			t_flag = 1;
			
		}
		else{
			if(t_flag == 0)
			{
				target_speed = -20.0f;
			}
			else{
				target_speed = 0.0f;
			}
		}
    /* set pwm pulse width */
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_pulse);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_pulse);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_pulse);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_pulse);
    
    /* system delay 1ms */
    HAL_Delay(2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/