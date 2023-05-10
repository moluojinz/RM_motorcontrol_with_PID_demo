//
// Created by ShiF on 2023/5/4.
//


#include "bsp_headfile.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "can.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
PID_t motor_speed_test;
Motor_measure_t MOTOR_Data;
uint32_t time;
uint32_t run_flag=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim5)//10ms
    {
//        time = Debug_Param().vel_rampTargetValue;
        motor_test_control();
//        usart_printf("%d\r\n",run_flag);
    }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)		//在默认得中断优先级配置时，按键中断里面写延时函数会卡死
{
    if(GPIO_Pin==KEY_Pin)
    {
        if(run_flag) run_flag=0;
        else run_flag=1;
    }
}
/* USER CODE END 4 */


void motor_Init(void)
{
    PID_SpeedParamInit(&motor_speed_test);
}

void motor_test_control(void)
{

//    motor_speed_test.Kp1=Debug_Param().vel_kp;
//    motor_speed_test.Ki1=Debug_Param().vel_ki;
//    motor_speed_test.Kd1=Debug_Param().vel_kd;
//    motor_speed_test.PID_Target= Debug_Param().vel_rampTargetValue;
    motor_speed_test.Kp1=(float)1.7;
    motor_speed_test.Ki1=(float)0.12;
    motor_speed_test.Kd1=(float)0.03;
    motor_speed_test.PID_Target= 456;

    PID_Update(&motor_speed_test,test_motor.speed);
    PID_GetPositionPID(&motor_speed_test);
//    run_flag = Debug_Param().vel_maxIntegral;
    if (run_flag != 1) {
        motor_speed_test.PID_Out = 0;
    }
    else{
    Set_motor1_cmd(&hcan1, (uint32_t) motor_speed_test.PID_Out);

    }
    usart_printf("%f,%d,%f,%d\r\n", motor_speed_test.PID_Out, test_motor.speed, motor_speed_test.PID_Target,run_flag);
}

