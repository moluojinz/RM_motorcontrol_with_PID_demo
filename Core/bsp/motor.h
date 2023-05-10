//
// Created by ShiF on 2023/5/4.
//

#ifndef MY_DEMO_MOTOR_H
#define MY_DEMO_MOTOR_H

#include "pid.h"

void motor_Init(void);
void motor_test_control(void);

extern PID_t motor_speed_test;


#endif //MY_DEMO_MOTOR_H
