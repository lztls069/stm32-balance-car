#include "motor.h"
#include "tim.h"
#include <stdlib.h>

void Load(int motorA, int motorB) {
    if (motorA < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // 设置PB13即AIN1为高电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 设置PB12即AIN2为低电平
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // 设置PB13即AIN1为低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // 设置PB12即AIN2为高电平
    }

    if (motorB < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // 设置PB14即BIN1为高电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // 设置PB15即BIN2为低电平
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // 设置PB14即BIN1为低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // 设置PB15即BIN2为高电平
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(motorA)); // 设置电机A的PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(motorB)); // 设置电机B的PWM占空比
}