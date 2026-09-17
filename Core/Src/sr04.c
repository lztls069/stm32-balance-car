#include "sr04.h"
#include "tim.h"
#include "pid.h"

uint32_t count; // 定义全局变量存储计数值
float distance; // 定义全局变量存储距离值

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_2){ // 检测到回波信号
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET){ // 上升沿
            __HAL_TIM_SET_COUNTER(&htim3, 0); // 清零计数器
            HAL_TIM_Base_Start(&htim3); // 启动定时器
        }
        else{ // 下降沿
            HAL_TIM_Base_Stop(&htim3); // 停止定时器
            count = __HAL_TIM_GET_COUNTER(&htim3); // 获取计数值
            distance = (count * 0.0343) / 2; // 计算距离，单位为厘米
            // 在这里可以使用distance变量进行后续处理，例如显示在OLED上
        }
    }

    if(GPIO_Pin == GPIO_PIN_5){
        Control();
    }
}

void Get_Distance(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // 发送高电平
    HAL_Delay(1); // 延时1毫秒
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // 发送低电平
}