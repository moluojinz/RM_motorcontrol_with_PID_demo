//
// Created by mjw on 2022/9/1.
//

#ifndef ROBO_TEST_CORE_INC_PID_H_
#define ROBO_TEST_CORE_INC_PID_H_

#include "stm32f4xx_hal.h"
#define PID_DEFAULT_PRECISION 0.0f		 //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 3000		 //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 10192	 //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192 //输出微分限幅

typedef enum
{
    IncrementPID_e = 0, //增量式PID
    PositionPID_e = 1,	//位置式PID
} PID_Type_e;

typedef enum
{
    Normal_e = 0,			////PID工作在正常状态
    Ramp_e = 1,				// PID工作在斜坡函数状态
    Ramping_Add = 2,		//斜坡还没完成，并且是加
    Ramping_Reduce = 3,		//斜坡还没完成，并且是减
    RampDefault_Step = 15, //斜坡函数步幅
    RampDefault_Time = 1,	//计数多少次才把count加上1个步幅
} PID_RampState_e;

typedef struct
{
    float Kp1;
    float Ki1;
    float Kd1;
    float PID_Err_now;
    float PID_Err_last;
    float PID_Err_lastlast;
    float PID_Err_all;
    float PID_Out;
    float PID_lastout;
    float PID_Target;
    float PID_Input;
    uint8_t State_RampOrNormal;
    float RampTarget;
    float RampCountTime;
    float RampTartgetTime; //计数多少次才把count加上1个步幅
    float RampTartgetStep; //斜坡函数步幅
    uint8_t PID_WorkType;  // PID工作在位置式还是增量式
    uint8_t Connected;
    float PID_Precision; // PID最小精度
    float PID_ErrAllMax; // PID积分限幅
    float PID_OutMax;	 // PID输出限幅
    float PID_OutStep;	 // PID输出步幅限制
} PID_t;

void PID_Update(PID_t *WhichPID, float NowInput);
float PID_GetPositionPID(PID_t *WhichPID);
float PID_GetIncrementalPID(PID_t *WhichPID);
void PID_SpeedParamInit(PID_t *WhichPID);
void PID_PosParamInit(PID_t *WhichPID);
void PID_SetTargetWithRamp(PID_t *WhichPID, float Tartget);
void PID_Clear(PID_t *WhichPID);
#endif //ROBO_TEST_CORE_INC_PID_H_
