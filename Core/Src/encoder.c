#include "encoder.h"

int Read_speed(TIM_HandleTypeDef *htim){
    int temp = (short)__HAL_TIM_GET_COUNTER(htim);//__HAL_TIM_GET_COUNTER(htim)获取定时器计数器的当前值(0~65535)，并将其强制转换为short类型(有符号16位整数)，然后赋值给temp变量。
    __HAL_TIM_SET_COUNTER(htim, 0);
    return temp;
}