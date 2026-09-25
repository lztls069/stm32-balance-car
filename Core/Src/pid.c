#include "main.h"
#include "pid.h"
#include "motor.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include "mpu6050.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "sr04.h"
#include <math.h>

#define SPEED_Y 12//前后最大速度
#define SPEED_Z 150//左右最大速度

//传感器数据变量
int Encoder_Left, Encoder_Right;
float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short accx, accy, accz;

//蓝牙操控
extern uint8_t fore, back, right, left;
extern TIM_HandleTypeDef htim2, htim4;
extern float distance;

//中间变量
int Vertical_out, Velocity_out, Turn_out, Target_speed, Target_turn, MOTO1, MOTO2;
float Med_Angle = -0.1;//机械中值

//PID参数 Velocity_Kp = -0.8, Velocity_Ki = -0.004
float Vertical_Kp = -3.9, Vertical_Kd = -0.0102;
float Velocity_Kp = -1, Velocity_Ki = -0.005;
/* 转向环：Turn_out = Turn_Sign * (Turn_Kp * Target_turn + Turn_Kd * gyroz)，再钳到 +-Turn_Out_Max
 * Turn_Kd 是常开阻尼增益，运行期不再被 Control() 改写（原来既当增益又当模式开关，根本调不动）。
 * 1) 稳定性：实测 d(gyroz)/dt = p * Turn_out 中 p>0（给 -23 的转向输出，100ms 内 gyroz 从 +12
 *    冲到 -1137），所以 Turn_Sign*Turn_Kd 必须为负，取 Turn_Sign = -1、Turn_Kd > 0 是正反馈→
 *    这里用 Turn_Sign = -1 配 Turn_Kd > 0 得到负反馈（实测原 Turn_Sign=+1 时车以 ±12° 剧振）。
 * 2) 方向：本车实测 gyroz 为正 = 实际左转（与旧笔记"向右为正"相反）。稳态角速度 ∝ -Kp/Kd*Target_turn，
 *    要"按右键右转"（Target_turn>0 时 gyroz<0）必须 Kp 与 Kd 同号，故 Kp > 0。翻 Kp 不影响阻尼。 */
float Turn_Kp = 0.2, Turn_Kd = 0.02;
int Turn_Out_Max = 40, Turn_Sign = -1;
/* 直行航向保持：Target_turn==0 时用 DMP yaw 把车拉回参考航向。
 * 纯电池供电、不插调试线时车会肉眼可见地持续左转（机械偏置 + 轮胎黏滞），
 * 而作用在角速度上的 Turn_Kd 在低速段被整数截断成 0，拦不住它——必须作用在角度上。
 * 只有偏出死区才给差速，并用独立限幅，避免和平衡环抢权威。 */
float Turn_Yaw_Kp = 2.0;     //PWM / 度
float Turn_Yaw_Dead = 1.5;   //死区，度
int   Turn_Yaw_Max = 20;     //航向修正独立限幅
uint8_t Turn_Yaw_Enable = 1; //0 = 暂停航向保持（静止诊断用）
uint8_t motor_enable = 1; //0 = 强制电机输出为 0（SWD STOP）
uint8_t tune_manual = 0;  //1 = Target_speed/Target_turn 交给 tune 模块，遥控按键失效
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
    static float yaw_ref = 0.0f;
    static uint8_t yaw_ref_ok = 0;
    float temp, trim = 0.0f;

    if (Target == 0.0f) {
        /* 直行：把航向偏差算成一个慢差速修正。yaw 正 = 实际左转（见上文），
         * 所以偏到左边(err>0)给正 trim -> 产生右转修正。 */
        if ((Turn_Yaw_Enable != 0u) && (yaw_ref_ok == 0u)) {
            yaw_ref = yaw;
            yaw_ref_ok = 1u;
        }
        if (Turn_Yaw_Enable != 0u) {
            float err = yaw - yaw_ref;
            if (err > 180.0f)  err -= 360.0f;
            if (err < -180.0f) err += 360.0f;
            if (err > Turn_Yaw_Dead)       trim = Turn_Yaw_Kp * (err - Turn_Yaw_Dead);
            else if (err < -Turn_Yaw_Dead) trim = Turn_Yaw_Kp * (err + Turn_Yaw_Dead);
            if (trim > (float)Turn_Yaw_Max)  trim = (float)Turn_Yaw_Max;
            if (trim < -(float)Turn_Yaw_Max) trim = -(float)Turn_Yaw_Max;
        }
    } else {
        yaw_ref_ok = 0u;   /* 转向中不保持，松开后重新捕获参考航向 */
    }

    temp = (float)Turn_Sign * (Turn_Kp * Target + Turn_Kd * gyro_Z + trim);
    /* 必须独立限幅：原来不限幅，差分直接顶到左右电机的 +-100 上，直立环会被拖垮 */
    if (temp > (float)Turn_Out_Max) temp = (float)Turn_Out_Max;
    if (temp < -(float)Turn_Out_Max) temp = -(float)Turn_Out_Max;
    return (int)temp;
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

    //倒地保护
    if (fabs(roll - Med_Angle) > 45) {
        stop = 1;
        Load(0, 0);
        return;
    }

    //遥控 前进
    if (tune_manual == 0) {
        if (fore == 0 && back == 0) {
            Target_speed = 0;
        }
        if (fore == 1) {
            Target_speed++;
        }
        if (back == 1) {
            Target_speed--;
        }
    }
    Target_speed = Target_speed > SPEED_Y ? SPEED_Y : (Target_speed < (-SPEED_Y) ? (-SPEED_Y) : Target_speed);//限制

    //左右
    if (tune_manual == 0) {
        if (right == 0 && left == 0) {
            Target_turn = 0;
        }
        if (right == 1) {
            Target_turn += 30;
        }
        if (left == 1) {
            Target_turn -= 30;
        }
    }
    Target_turn = Target_turn > SPEED_Z ? SPEED_Z : (Target_turn < (-SPEED_Z) ? (-SPEED_Z) : Target_turn);//限制

    //2 将数据传入PID控制器计算输出结果
    Velocity_out = Velocity(Target_speed, Encoder_Left, Encoder_Right); // 计算速度环输出
    Vertical_out = Vertical(Velocity_out + Med_Angle, roll, gyrox); // 计算直立环输出
    PWM_out = Vertical_out; // 将直立环输出作为PWM输出
    Turn_out = Turn(Target_turn, gyroz); // 计算转向环输出
    MOTO1 = PWM_out - Turn_out; // 计算电机1的输出
    MOTO2 = PWM_out + Turn_out; // 计算电机2的输出
    Limit(&MOTO1, &MOTO2); // 限制电机输出范围
    if (motor_enable == 0) { //SWD STOP：下一拍即生效
        MOTO1 = 0;
        MOTO2 = 0;
    }
    Load(MOTO1, MOTO2); // 将计算结果加载到电机驱动器
}
