#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "pid.h"


void updateRSRA(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra);
void send(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra);