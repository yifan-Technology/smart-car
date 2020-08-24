#include "serial.h"


uint8_t rData[18];  //  for saving RX Data
uint8_t rDataCount = 0;  //  count Data bytes


void updateRSRA(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra)
{	
  // get current rotate spped and rotate angle
  
  *real_left_front_rs = moto_chassis[0].speed_rpm;
  *real_left_front_ra = *soll_left_front_rs;

  *real_right_front_rs = moto_chassis[1].speed_rpm;
  *real_right_front_ra = *soll_right_front_rs;
  
  *real_left_back_rs = -moto_chassis[2].speed_rpm;
  *real_left_back_ra = *soll_left_back_rs;

  *real_right_back_rs = -moto_chassis[3].speed_rpm;
  *real_right_back_ra = *soll_right_back_rs;
}

void send(float *set_spd, float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra)
{	 
	if (rDataFlag==1) {
		//HAL_UART_Transmit(&huart6, rDataBuffer, sizeof(rDataBuffer), 0xffff);
		rDataFlag = 0;
		
		uint8_t a[4];
		for (int i = 0; i < 4; i++) {
			a[i] = rData[i+1];
		}
		soll_left_front_rs = (float *)a;
		
		uint8_t b[4];
		for (int i = 0; i < 4; i++) {
			 b[i] = rData[i+5];
		}
		soll_right_front_rs = (float *)b;

		uint8_t c[4];
		for (int i = 0; i < 4; i++) {
			c[i] = rData[i+9];
		}
		soll_left_back_rs = (float *)c;
		
		uint8_t d[4];
		for (int i = 0; i < 4; i++) {
			 d[i] = rData[i+13];
		}
		soll_right_back_rs = (float *)d;
		
		set_spd[0] = (*soll_left_front_rs);
    set_spd[1] = (*soll_left_back_rs);
    set_spd[2] = (*soll_right_front_rs);
		set_spd[3] = (*soll_right_back_rs);
		
		updateRSRA(real_left_front_rs, real_left_front_ra, real_right_front_rs, real_right_front_ra, real_left_back_rs, real_left_back_ra, real_right_back_rs, real_right_back_ra);

		uint8_t send_info[] = {0x61,0x61,0x61,0x61, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x62,0x62,0x62,0x62};
			
		*(float*)(send_info+4) = *real_left_front_rs;
		*(float*)(send_info+8) = *real_left_front_ra;
		*(float*)(send_info+12) = *real_right_front_rs;
		*(float*)(send_info+16) = *real_right_front_ra;
		*(float*)(send_info+20) = *real_left_back_rs;
		*(float*)(send_info+24) = *real_left_back_ra;
		*(float*)(send_info+28) = *real_right_back_rs;
		*(float*)(send_info+32) = *real_right_back_ra;

		HAL_UART_Transmit(&huart6, send_info, sizeof(send_info), 0xffff);
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	if (rDataBuffer[0] == 0x63)		rDataCount = 0;
  rData[rDataCount]=rDataBuffer[0];
	rDataCount++;
  
	if(rDataBuffer[0]==0x64){  
		rDataCount = 0;
		rDataFlag = 1;
		
	}
  
  while(HAL_UART_Receive_IT(&huart6, rDataBuffer, 1) != HAL_OK); // Wait completly receive 1 byte data, and put data in rDataBuffer
}