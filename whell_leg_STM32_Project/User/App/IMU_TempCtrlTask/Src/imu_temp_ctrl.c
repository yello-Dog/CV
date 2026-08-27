#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "imu_temp_ctrl.h"

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float temp;
uint8_t forceStop = 0;


float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

const IMU_Data_t* IMU_DATA_TEMP_HANDLE;
/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
extern uint8_t BMI088_INIT_OK_FLAG;

void IMU_TempCtrltask(void)
{
    osDelay(500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		IMU_DATA_TEMP_HANDLE = BMI088_GetData();
    while(BMI088_INIT_OK_FLAG == 0)
    {
        ;
    }
    for (;;)
    {
				temp = IMU_DATA_TEMP_HANDLE->Temperature;
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        // 有保护的
				if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        
        if (forceStop == 1)
        {
            out = 0.0f;
        }
        
        htim3.Instance->CCR4 = (uint16_t)out;
				
				osDelay(20);
    }
}


