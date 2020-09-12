#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "pid.h"

static float * soll_left_front_rs;
static float * soll_left_back_rs;
static float * soll_right_front_rs;
static float * soll_right_back_rs;


static uint8_t rDataBuffer[1];  //  RX Data buffer
static uint8_t rDataFlag = 0;  //  waitting complete RX date having been send 

#define RX_MODE()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET)
#define TX_MODE()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET)

void updateRSRA(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra);
void send(float *real_left_front_rs, float *real_left_front_ra, float *real_right_front_rs, float *real_right_front_ra, float *real_left_back_rs, float *real_left_back_ra, float *real_right_back_rs, float *real_right_back_ra);