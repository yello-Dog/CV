#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H


#include "stdint.h"
#include "ins_task.h"

#include "main.h"
#include "kalman_filter.h"


extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern const float *Observer_Get_xdot_xdotdot(void);
#endif




