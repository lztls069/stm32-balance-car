#include "pid.h"

float Vertical_Kp, Vertical_Kd;

//直立环
//输入：当前角度，目标角度（机械中值），当前角速度
int Vertical(float Med, float Angle, float gyro_Y)
{
    int temp;
    temp = (int)(Vertical_Kp * (Angle - Med) + Vertical_Kd * (gyro_Y));
    return temp;
}