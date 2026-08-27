/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "cmsis_os2.h"
#include "ps2_drv.h"
#include "BMI088driver.h"
#include "INS_task.h"
#include "chassisL_task.h"
#include "chassisR_task.h"
#include "observe_task.h"
#include "DebugConfig.h"
#include "imu_temp_ctrl.h"
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
/* USER CODE BEGIN Variables */

osThreadId_t INS_TASKHandle;
osThreadId_t R_CHASSIS_TASKHandle;
osThreadId_t L_CHASSIS_TASKHandle;
osThreadId_t OBSERVER_TASKHandle;
osThreadId_t PS2_TASKHandle;
osThreadId_t Temp_TASKHandle;

extern SPI_HandleTypeDef hspi2;
volatile uint32_t IF_RTOS_UPDATE_DATA = 0;
volatile uint32_t fm_magic = 0x12345678;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void FREERTOS_ALLTASK_Init(void);

// ���ӵ���Ӧ�ĺ���ʵ������
void INS_Task(void*argument);
void R_CHASSIS_Task(void*argument);
void L_CHASSIS_Task(void*argument);
void OBSERVER_Task(void*argument);
void PS2_Task(void*argument);
void IMU_TempCtrlTask(void*argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	FREERTOS_ALLTASK_Init();
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
		#if DEBUGGING_WITHOUT_IMU
		IF_RTOS_UPDATE_DATA++;
		#endif
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void FREERTOS_ALLTASK_Init()
{
	//--------------------------------------------------------------------	
	const osThreadAttr_t INS_TASK_attributes = {
		.name = "INS_TASK",
		.stack_size = 512,
		.priority = (osPriority_t) osPriorityRealtime,
	};

	INS_TASKHandle = osThreadNew(INS_Task, NULL, &INS_TASK_attributes);
	//--------------------------------------------------------------------	

	const osThreadAttr_t R_CHASSIS_TASK_attributes = {
		.name = "R_CHASSIS_TASK",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityAboveNormal,
	};

	R_CHASSIS_TASKHandle = osThreadNew(R_CHASSIS_Task, NULL, &R_CHASSIS_TASK_attributes);
	//--------------------------------------------------------------------	
	const osThreadAttr_t L_CHASSIS_TASK_attributes = {
		.name = "L_CHASSIS_TASK",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityAboveNormal,
	};

	L_CHASSIS_TASKHandle = osThreadNew(L_CHASSIS_Task, NULL, &L_CHASSIS_TASK_attributes);
	//--------------------------------------------------------------------	

	const osThreadAttr_t OBSERVER_TASK_attributes = {
		.name = "OBSERVER_TASK",
		.stack_size = 512,
		.priority = (osPriority_t) osPriorityHigh,
	};

	OBSERVER_TASKHandle = osThreadNew(OBSERVER_Task, NULL, &OBSERVER_TASK_attributes);
	//--------------------------------------------------------------------	
	const osThreadAttr_t PS2_TASK_attributes = {
		.name = "PS2_TASK",
		.stack_size = 512,
		.priority = (osPriority_t) osPriorityAboveNormal,
	};

	PS2_TASKHandle = osThreadNew(PS2_Task, NULL, &PS2_TASK_attributes);
	
		const osThreadAttr_t Temp_TASK_attributes = {
		.name = "Temp_TASK",
		.stack_size = 256,
		.priority = (osPriority_t) osPriorityNormal,
	};

	Temp_TASKHandle = osThreadNew(IMU_TempCtrlTask, NULL, &Temp_TASK_attributes);
	//--------------------------------------------------------------------	
}

void INS_Task(void *argument )
{


  	#if DEBUGGING_WITHOUT_IMU
		while(1)
		{
			osDelay(1);
		}
		#else
		// 1ms
    INS_task();		
		#endif
	
}

void R_CHASSIS_Task(void *argument )
{

  
		// 最少3ms
	#if DEBUGGING_WITHOUT_IMU
	while(1)
	{

    /*
     * main 中已经完成：
     * MX_FDCAN2_Init();
     * FDCAN2_Config();
     */

   

    /* 完全照达妙原程序：发 10 次使能帧 */
    for (uint8_t i = 0; i < 10; i++)
    {
        enable_motor_mode(&hfdcan2, 0x06, MIT_MODE);
        osDelay(1);
    }

    /* 完全照达妙未启动状态：持续发送全零 MIT 帧 */
    for (;;)
    {
        mit_ctrl(&hfdcan2,
                 0x06,
                 0.0f,   // pos
                 0.0f,   // vel
                 0.0f,   // kp
                 0.0f,   // kd
                 0.0f);  // torque

        osDelay(3);

		}
	}
	#else
  ChassisR_task();
  #endif

}

void L_CHASSIS_Task(void *argument )
{


  	#if (DEBUGGING_WITHOUT_IMU == 1)
		while(1)
		{
			osDelay(1);
		}
		#else
		// 最少是3ms
    ChassisL_task();
		#endif

}

void OBSERVER_Task(void *argument )
{


    #if DEBUGGING_WITHOUT_IMU
		while(1)
		{
			osDelay(1);
		}
		#else
		// 里面任务周期是3ms
    Observe_task();
    #endif

}

void PS2_Task(void *argument )
{


    #if DEBUGGING_WITHOUT_IMU
		while(1)
		{
			osDelay(1);
		}
		#else
		// 读取周期是10ms
    PS2_task();
    #endif

}

void IMU_TempCtrlTask(void * argument)
{
  /* USER CODE BEGIN IMU_TempCtrlTask */
  /* Infinite loop */
	IMU_TempCtrltask();
  /* USER CODE END IMU_TempCtrlTask */
}

/* USER CODE END Application */

