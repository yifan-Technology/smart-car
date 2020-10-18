#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "tim.h"
#include "serial.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

#define CAN_CONTROL	//const current control 

int set_v,set_spd[4];
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	
	HAL_UART_Receive_IT(&huart6,RxBuffer,10);
	
//	PID_struct_init(&pid_omg, POSITION_PID, 20000, 20000,
//									1.5f,	0.1f,	0.0f	);  //angular rate closeloop.
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
	
	HAL_Delay(100);
  /* Infinite loop */
  for(;;)
  {
		
		
		  
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		}
		set_moto_current(&hcan1, pid_spd[0].pos_out, 
								pid_spd[1].pos_out,
								pid_spd[2].pos_out,
								pid_spd[3].pos_out);
		
		set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = 500;
		  
		RX_MODE();
    HAL_Delay(500);
    TX_MODE();
    send();
    HAL_Delay(500);
		
		//osDelay(10);
		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

