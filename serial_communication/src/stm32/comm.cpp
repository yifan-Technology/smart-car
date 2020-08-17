#include "comm.h"







// void vuser_send_string(UART_HandleTypeDef *handle, unsigned char *ubyte){
// 	while (*ubyte) {
// 		HAL_UART_Transmit(handle, ubyte, 1, 0xff);
// 		ubyte++;
// 	}
// }

// void vuser_receive_data(UART_HandleTypeDef *handle) {
// 	unsigned char ubyte[] = "";
// 	HAL_UART_Receive(handle, ubyte, 1, 0xff);
// 	//while (HAL_UART_GetState(handle) == HAL_UART_STATE_BUSY){}
// 	HAL_UART_Transmit(handle, ubyte, sizeof(ubyte), 0xff);
// }

// void vuser_transmit_DMA(UART_HandleTypeDef *uartHandle, const char *format, ...)
// {
// 	static char buf[120];
// 	va_list ap;
// 	va_start(ap, format);
// 	while(HAL_UART_GetState(uartHandle) == HAL_UART_STATE_BUSY_TX){}
// 	if (vsprintf(buf, format, ap) > 0)
// 	{
// 		HAL_UART_Transmit_DMA(uartHandle, (uint8_t *)buf, sizeof(buf));
// 	}
// 	va_end(ap);
// }
	
// uint8_t ubyte[10];

// void vuser_receive_data_IT(UART_HandleTypeDef *handle)
// {
// 	HAL_UART_Receive_IT(handle, (uint8_t*)ubyte, 1);
// }

// void vuser_receive_DMA(UART_HandleTypeDef *uartHandle)
// {
// 	HAL_UART_Receive_DMA(uartHandle, (uint8_t*)ubyte, 1);
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
// {
// 	uint8_t myBuffer[] = "I have gotten your message: ";
// 	//uint8_t Enter[] = "\r\n";
// 	//while(HAL_UART_GetState(handle) == HAL_UART_STATE_BUSY_TX){}
// 	//while(HAL_UART_Transmit(handle, (uint8_t*)myBuffer, sizeof(myBuffer), 5000) != HAL_OK);
// 	//while(HAL_UART_Transmit(handle, (uint8_t*)ubyte, sizeof(ubyte), 5000) != HAL_OK);
// 	//while(HAL_UART_Transmit(handle, (uint8_t*)Enter, sizeof(Enter), 5000) != HAL_OK);
// 	HAL_UART_Receive_DMA(handle, (uint8_t*)ubyte, 1);
// }	


