
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "comm.h"
#include "string.h"
#include "stdlib.h"

#define RX_MODE()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define TX_MODE()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

uint8_t RxBuffer[10];
uint8_t *Rx_Count;
float * soll_left_rs;
float * soll_right_rs;

//struct ControlFrame controlFrame;
//struct FeedBackFrame feedBackFrame;
//struct ControlData controlData;
//struct FeedBackData feedBackData;

void SystemClock_Config(void);

void init()
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
}

void DoPID()
{

}

// struct FeedBackData GetRSRA()
// {
//   // get current rotate spped and rotate angle
// 	struct FeedBackData feedBackData;
// 	feedBackData.real_left_rs = 1.1;
// 	feedBackData.real_left_ra = 1.2;
// 	feedBackData.real_right_rs = 1.3;
// 	feedBackData.real_right_ra = 1.4;

//   return feedBackData;
// }

// FeedBackFrame pack(FeedBackData feedBackData){
//    FeedBackFrame feedBackFrame =
//   {
//     STM32CommSOF,
//     feedBackData.real_left_rs, 
//     feedBackData.real_left_ra, 
//     feedBackData.real_right_rs,
//     feedBackData.real_right_ra,
//     STM32CommEOF
//   };
//   return feedBackFrame;
// }

void updateRSRA(float *real_left_rs, float *real_left_ra, float *real_right_rs, float *real_right_ra)
{	
	// get current rotate spped and rotate angle
  if (soll_left_rs == NULL) {
    *real_left_rs = 0.0;
  } else {
    *real_left_rs = soll_left_rs[0];
  }
	// *real_left_rs = soll_left_rs == NULL ? 0.0 : soll_left_rs[0];
	*real_left_ra = 112.245;

  if (soll_right_rs == NULL) {
    *real_right_rs = 0.0;
  } else {
    *real_right_rs = soll_right_rs[0];
  }
	// *real_right_rs = soll_right_rs == NULL ? 0.0 : soll_right_rs[0];
	*real_right_ra = 113.467;
}

void send()
{	
	float real_left_rs;  
	float real_left_ra; 
	float real_right_rs;          
	float real_right_ra;  
	updateRSRA(&real_left_rs, &real_left_ra, &real_right_rs, &real_right_ra);

  uint8_t send_info[] = {STM32CommSOF,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,STM32CommEOF};
		
	*(float*)(send_info+4) = real_left_rs;
	*(float*)(send_info+8) = real_left_ra;
	*(float*)(send_info+12) = real_right_rs;
	*(float*)(send_info+16) = real_right_ra;

  // int len = sizeof(send_info);
  // static char str[20];
  // sprintf(str, "aa: %d\n", len);
  // HAL_UART_Transmit(&huart1, (uint8_t *)str, sizeof(str), 0xffff);

  HAL_UART_Transmit(&huart1, send_info, sizeof(send_info), 0xffff);
	
	// if (send_info[0] == 0x05){
 //    char a[4];
 //    for (int i = 0; i < 4; i++) {
 //        a[i] = send_info[i+4];
 //    }
 //    float * aa = (float *)a;
 //    static char str[20];
 //    sprintf(str, "aa: %f\n", aa[0]);
 //    HAL_UART_Transmit(&huart1, (uint8_t *)str, sizeof(str), 0xffff);
      
 //  }else{
 //    char b[4];
 //    for (int i = 0; i < 4; i++) {
 //        b[i] = send_info[i+8];
 //    }
 //    float * bb = (float *)b;
 //    static char strr[20];
 //    sprintf(strr, "aa: %f\n", bb[0]);
 //    HAL_UART_Transmit(&huart1, (uint8_t *)strr, sizeof(strr), 0xffff);
 //  }
  
	
 //  if (send_info[23] == 0x06) {
 //    char c[4];
 //    for (int i = 0; i < 4; i++) {
 //        c[i] = send_info[i+12];
 //    }
 //    float * cc = (float *)c;
 //    static char strrr[20];
 //    sprintf(strrr, "aa: %f\n", cc[0]);
 //    HAL_UART_Transmit(&huart1, (uint8_t *)strrr, sizeof(strrr), 0xffff);
 //  } else{
 //    char d[4];
 //    for (int i = 0; i < 4; i++) {
 //        d[i] = send_info[i+16];
 //    }
 //    float * dd = (float *)d;
 //    static char strrrr[20];
 //    sprintf(strrrr, "aa: %f\n", dd[0]);
 //    HAL_UART_Transmit(&huart1, (uint8_t *)strrrr, sizeof(strrrr), 0xffff);
 //  }
  
}

int main(void)
{	
	init();
	
	//set data frame SOF and EOF
	// struct ControlFrame controlFrame;
	// struct FeedBackFrame feedBackFrame;
	// controlFrame.SOF = JetsonCommSOF;
	// controlFrame._EOF = JetsonCommEOF;
	// feedBackFrame.SOF = STM32CommSOF;
	// feedBackFrame._EOF = STM32CommEOF;
  
	// struct ControlData controlData;
	// struct FeedBackData feedBackData;

	
	//vuser_receive_DMA(&huart1);
	//unsigned char want_send[] = "Hello World.";

	// uint8_t txbuf[50];
	// memcpy(txbuf,"qwertyu\n",50);
 //  TX_MODE();
 //  HAL_UART_Transmit(&huart1,txbuf,strlen((char *)txbuf),1000);
  
 //  memcpy(txbuf,"asdfgh\n",50);
 //  HAL_UART_Transmit(&huart1,txbuf,strlen((char *)txbuf),1000);
 //  RX_MODE();

  //RX_MODE();
  HAL_UART_Receive_IT(&huart1,RxBuffer,10);
	
  //RX_MODE();
	
  while (1)
  {
		//vuser_receive_data(&huart1);
		//vuser_transmit_DMA(&huart1, "dma works!");
		//vuser_send_string(&huart1, want_send);
		
		// TX_MODE();
  //   HAL_UART_Transmit(&huart1,txbuf,strlen((char *)txbuf),1000);
		// HAL_Delay(1000);
  //   RX_MODE();
		// HAL_Delay(500);

    RX_MODE();
    DoPID();
    HAL_Delay(500);
    TX_MODE();
    send();
    HAL_Delay(500);
		// if (soll_left_rs != NULL) {
		// 	static char str[10];
  //     sprintf(str, "soll %f\n", soll_left_rs[0]);
  //     HAL_UART_Transmit(&huart1,(uint8_t *)str,10,10);
		// }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{	
  if (1 == 1) {
	//if (RxBuffer[0] == JetsonCommSOF && RxBuffer[9] == JetsonCommEOF) {
		uint8_t a[4];
    for (int i = 0; i < 4; i++) {
       a[i] = RxBuffer[i+1];
    }
    soll_left_rs = (float *)a;
    // static char str[10];
    // sprintf(str, "%f\n", soll_left_rs[0]);
    // HAL_UART_Transmit(&huart1,(uint8_t *)str,10,10);

    uint8_t b[4];
    for (int i = 0; i < 4; i++) {
       b[i] = RxBuffer[i+5];
    }
    soll_right_rs = (float *)b;
    // static char strr[10];
    // sprintf(strr, "%f\n", soll_right_rs[0]);
    // HAL_UART_Transmit(&huart1,(uint8_t *)strr,10,10);
 
	}
  
  // HAL_UART_Transmit(&huart1,RxBuffer,10,10);
  HAL_UART_Receive_IT(&huart1,RxBuffer,10);
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
