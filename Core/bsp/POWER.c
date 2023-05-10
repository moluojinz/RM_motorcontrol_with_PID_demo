//
// Created by ShiF on 2023/5/4.
//
#include "stm32f4xx_hal.h"

void __POWER_24V_ON(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
}
