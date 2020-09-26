#include "serial.h"

//extern float set_spd[4];


void updateRSRA(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra)
{	
  // get current rotate spped and rotate angle
  
	//*real_left_front_rs = set_spd[0];
	*real_left_front_rs = moto_chassis[0].speed_rpm;
	*real_left_front_ra = moto_chassis[0].last_angle;
	
	//*real_right_front_rs = set_spd[1];
	*real_right_front_rs = -moto_chassis[2].speed_rpm;
	*real_right_front_ra = moto_chassis[2].last_angle;
	
	//*real_left_back_rs = set_spd[2];
	*real_left_back_rs = moto_chassis[1].speed_rpm;
	*real_left_back_ra = moto_chassis[1].last_angle;
	
	//*real_right_back_rs = set_spd[3];
	*real_right_back_rs = -moto_chassis[3].speed_rpm;
	*real_right_back_ra = moto_chassis[3].last_angle;
}

void send(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra)
{	 	
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
	while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX){}
	HAL_UART_Transmit_DMA(&huart6, send_info, sizeof(send_info));
}