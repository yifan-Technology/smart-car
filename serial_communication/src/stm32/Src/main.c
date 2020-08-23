#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "serial.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);


void init()
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART6_UART_Init();
	
	MX_CAN1_Init();
	
	/* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();
}

float set_spd[4];
float real_left_front_rs = 0.0;  
float real_left_front_ra = 0.0; 
float real_right_front_rs = 0.0;          
float real_right_front_ra = 0.0; 
float real_left_back_rs = 0.0;  
float real_left_back_ra = 0.0; 
float real_right_back_rs = 0.0;          
float real_right_back_ra = 0.0; 

int main(void)
{
	init();
	
	
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
	
	float a = 0;
	float b = 0;
	float c = 0;
	float d = 0;
	
	soll_left_front_rs = &a;
	soll_left_back_rs = &c;
	soll_right_front_rs = &b;
	soll_right_back_rs = &d;

	set_spd[0] = (*soll_left_front_rs);
	set_spd[1] = (*soll_left_back_rs);
	set_spd[2] = (*soll_right_front_rs);
	set_spd[3] = (*soll_right_back_rs);
	
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	while(HAL_UART_Receive_IT(&huart6, rDataBuffer, 1) != HAL_OK);
	
	HAL_Delay(100);
	
  while (1)
  {
		send(set_spd, &real_left_front_rs, &real_left_front_ra, &real_right_front_rs, &real_right_front_ra, &real_left_back_rs, &real_left_back_ra, &real_right_back_rs, &real_right_back_ra);
		
		for(int i=0; i<2; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		}
		
		for(int i=2; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, -set_spd[i]);
		}
		
		set_moto_current(&hcan1, pid_spd[0].pos_out, 
								pid_spd[1].pos_out,
								pid_spd[2].pos_out,
								pid_spd[3].pos_out);
		
		HAL_Delay(10);
		
//    HAL_Delay(5);
//    TX_MODE();
//    HAL_Delay(5);
//		RX_MODE();
  }
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
