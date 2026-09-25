#ifndef PID_H
#define PID_H

#include <stdint.h>

int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_L, int encoder_R);
int Turn(float Target, float gyro_Z);
void Control(void);

/* tune.c（SWD 在线调参）需要读写这些量 */
extern float Vertical_Kp, Vertical_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;
extern int   Turn_Out_Max, Turn_Sign;
extern float Turn_Yaw_Kp, Turn_Yaw_Dead;
extern float Turn_Yaw_Kd;
extern int   Turn_Yaw_Max;
extern uint8_t Turn_Yaw_Enable;
extern int   Vertical_out, Velocity_out, Turn_out, Target_speed, Target_turn, MOTO1, MOTO2;
extern float pitch, roll, yaw, Med_Angle;
extern short gyrox, gyroy, gyroz;
extern int   Encoder_Left, Encoder_Right;
extern uint8_t motor_enable, tune_manual;

#endif
