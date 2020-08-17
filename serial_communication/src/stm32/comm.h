#ifndef __USER_CONNECT_H_
#define __USER_CONNECT_H_

#ifdef __cplusplus
extern "C" {
#endif

	#include "usart.h"
	#include "stm32f4xx_it.h"
	#include "stdarg.h"
	#include "stdio.h"

	// void vuser_send_string(UART_HandleTypeDef *handle, unsigned char *ubyte);
	// void vuser_receive_data(UART_HandleTypeDef *handle);
	// void vuser_receive_data_IT(UART_HandleTypeDef *handle);
	// void vuser_transmit_DMA(UART_HandleTypeDef *uartHandle, const char *format, ...);
	// void vuser_receive_DMA(UART_HandleTypeDef *uartHandle);
	
	typedef struct ControlData
	{
	    float   soll_left_rs;           
	    float   soll_right_rs;          
	}ControlData;

	typedef struct FeedBackData
	{
	    float   real_left_rs;           
	    float   real_right_rs;          
	    float   real_left_ra;           
	    float   real_right_ra;          
	}FeedBackData;

	enum
	{
			JetsonCommSOF = (uint8_t)0x03,
			JetsonCommEOF = (uint8_t)0x04,
			STM32CommSOF = (uint8_t)0x05,
			STM32CommEOF = (uint8_t)0x06,
	};

	typedef struct ControlFrame
	{
			uint8_t  SOF;
			float    soll_left_rs;           
			float    soll_right_rs;          
			uint8_t  _EOF;
	}ControlFrame;

	typedef struct FeedBackFrame
	{
			uint8_t  SOF;
			float    real_left_rs;           
			float    real_right_rs;          
			float    real_left_ra;           
			float    real_right_ra;          
			uint8_t  _EOF;
	}FeedBackFrame;
	
	
#ifdef __cplusplus
}
#endif
#endif
