#ifndef PID_H
#define PID_H

int Vertical(float Med, float Angle, float gyro_Y);
int Velocity(int Target, int encoder_L, int encoder_R);
int Turn(float Target, float gyro_Z);
void Control(void);
#endif