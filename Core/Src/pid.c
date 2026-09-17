#include "pid.h"
#include "motor.h"
#include "tim.h"
#include "mpu6050.h"
#include "encoder.h"
#include "inv_mpu.h"

//传感器数据变量
int Encoder_Left, Encoder_Right;
float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short accx, accy, accz;

//中间变量
int Vertical_out, Velocity_out, Turn_out, Target_speed, Target_turn, MOTO1, MOTO2;
float Med_Angle = 0.2;//机械中值

//PID参数 Velocity_Kp = -0.8, Velocity_Ki = -0.004
float Vertical_Kp = -3.9, Vertical_Kd = -0.0102;
float Velocity_Kp = -1, Velocity_Ki = -0.005;
float Turn_Kp, Turn_Kd;
uint8_t stop = 0; //速度环积分清零标志

//直立环 mpu6050已经滤过波不用软件滤波
//输入：当前角度，目标角度（机械中值），当前角速度
int Vertical(float Med, float Angle, float gyro_Y)
{
    int temp;
    temp = (int)(Vertical_Kp * (Angle - Med) + Vertical_Kd * (gyro_Y));
    return temp;
}

//速度环PI控制器 编码器外设没有滤波需要软件低通滤波
//输入：期望速度、左编码器、右编码器
int Velocity(int Target, int encoder_L, int encoder_R)
{
    static int Err_LowOut_last, Encoder_S;
    static float a=0.7;
    int Err, Err_LowOut, temp;
    //1、计算速度误差 = (左轮实际速度 + 右轮实际速度) - 目标速度 
    Err=(encoder_L+encoder_R)-Target;
    //2、低通滤波 对误差进行低通滤波（平滑噪声）
    Err_LowOut=(1-a)*Err + a*Err_LowOut_last;
    Err_LowOut_last=Err_LowOut;
    //3、积分 将滤波后的误差累加到积分项
    Encoder_S+=Err_LowOut;
    //4、积分限幅(-20000~20000) 对积分项进行限幅（防止积分饱和），特殊情况（stop）清零
    Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
    if(stop == 1) Encoder_S = 0, stop = 0;
    //5、速度环计算 计算PI输出 = Kp * 滤波后误差 + Ki * 积分项
    temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
    //6、限制temp
    if (temp > 10)  temp = 10;
    if (temp < -10) temp = -10;

    return temp;
}

//转向环PD控制器
//输入：期望角度，角速度
int Turn(float Target, float gyro_Z)
{
    int temp;
    temp = Turn_Kp * Target + Turn_Kd * gyro_Z;
    return temp;
}

//控制
void Control(){
    int PWM_out;
    //1 读取编码器和陀螺仪
    Encoder_Left = Read_speed(&htim2); // 读取定时器2的编码器计数值
    Encoder_Right = -Read_speed(&htim4); // 读取定时器4的编码器计数值
    mpu_dmp_get_data(&pitch, &roll, &yaw); // 获取MPU6050的姿态数据
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); // 获取MPU6050的陀螺仪数据
    MPU_Get_Accelerometer(&accx, &accy, &accz); // 获取MPU6050的加速度数据

    //2 将数据传入PID控制器计算输出结果
    Velocity_out = Velocity(Target_speed, Encoder_Left, Encoder_Right); // 计算速度环输出
    Vertical_out = Vertical(Velocity_out + Med_Angle, roll, gyrox); // 计算直立环输出
    PWM_out = Vertical_out; // 将直立环输出作为PWM输出
    Turn_out = Turn(Target_turn, gyroz); // 计算转向环输出
    MOTO1 = PWM_out - Turn_out; // 计算电机1的输出
    MOTO2 = PWM_out + Turn_out; // 计算电机2的输出
    Limit(&MOTO1, &MOTO2); // 限制电机输出范围
    Load(MOTO1, MOTO2); // 将计算结果加载到电机驱动器
}