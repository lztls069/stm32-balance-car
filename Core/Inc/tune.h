/**
 * @file    tune.h
 * @brief   SWD 在线调参 mailbox：片上抓波形 + 运行期改 PID 参数。
 *
 * 主机侧（STM32CubeProgrammer CLI）用法：
 *     -c port=SWD mode=hotplug -r32 <g_tune> <n>     读 mailbox
 *     -c port=SWD mode=hotplug -w32 <g_tune> <v0> ... 写 mailbox
 *
 * 关键约定：主机写入区是一段连续、按地址递增的 int32。`cmd` 是该区最后一个字，
 * 因此一次递增的 -w32 写天然保证「参数先落地、命令后生效」。
 * 目标在消费完命令后把 cmd 清零，所以同一条命令可以连续下发。
 *
 * 定点缩放一律用 x1e6（微单位），保证 -0.0102 这类参数往返无损：
 * 例如 Turn_Kp = -0.08 -> -80000，Vertical_Kd = -0.0102 -> -10200。
 *
 * arg1 高 16 位复用为斜坡拍数（采样点数 <= 500，装得下）：
 *     arg1 = (ramp_ticks << 16) | samples
 * ramp_ticks = 0 表示硬阶跃；>0 表示 Target_turn 用这么多拍(10ms/拍)线性升到 step_amp。
 *
 * 采样由 MPU6050 的 EXTI 中断（Control() 所在处）按 10ms 节拍推进，不占用主循环。
 */
#ifndef TUNE_H
#define TUNE_H

#include <stdint.h>
#include <stddef.h>

#define TUNE_MAGIC        0x544E3031u   /* "TN01" */
#define TUNE_BUF_SAMPLES  500u          /* 500 x 10ms = 5s */
#define TUNE_BUF_CH       8u
#define TUNE_TRIAL_MAX    20u
#define TUNE_TRIAL_COLS   12u

/* cmd 取值 */
#define TUNE_CMD_IDLE     0
#define TUNE_CMD_APPLY    1   /* 仅应用参数 */
#define TUNE_CMD_ARM      2   /* 武装采集，无转向阶跃（用于手推阻尼测试） */
#define TUNE_CMD_STOP     3   /* motor_enable = 0，并释放手动模式 */
#define TUNE_CMD_STEP     4   /* 武装采集，带转向阶跃 */
#define TUNE_CMD_CLRTRIAL 5   /* 清空试次表 */
#define TUNE_CMD_CLRSTAT  6   /* 清空耗时/错误计数 */

/* 采集通道索引 */
#define TUNE_CH_GYROZ  0
#define TUNE_CH_TTURN  1
#define TUNE_CH_TOUT   2
#define TUNE_CH_MOTO1  3
#define TUNE_CH_MOTO2  4
#define TUNE_CH_ROLL   5   /* 单位 0.01 度 */
#define TUNE_CH_ENCL   6
#define TUNE_CH_ENCR   7

/* state 位 */
#define TUNE_ST_ARMED  0x01u
#define TUNE_ARG1(samples, ramp)  ((((int32_t)(ramp)) << 16) | ((int32_t)(samples) & 0xFFFF))
#define TUNE_ARG1_SAMPLES(a)      ((int32_t)((a) & 0xFFFF))
#define TUNE_ARG1_RAMP(a)         ((int32_t)(((a) >> 16) & 0xFFFF))

#define TUNE_ST_FULL   0x02u
#define TUNE_ST_FELL   0x04u
#define TUNE_ST_MANUAL 0x08u

/* 试次行 flags 位 */
#define TUNE_TR_FELL     0x01    /* 采集期间倒地 */
#define TUNE_TR_DIVERGE  0x02    /* 偏航角速度发散（符号错或增益过大） */
#define TUNE_TR_NOSIGNAL 0x04    /* 稳态角速度近乎为 0，指标不可用 */
#define TUNE_TR_NEGSTEP  0x08    /* 阶跃为负方向 */

typedef struct {
    /* ---------------- 主机 -> 目标（一次递增 -w32 写完） ---------------- */
    volatile int32_t arg0;             /* 0x00 采集延时 ms（默认 500） */
    volatile int32_t arg1;             /* 0x04 采集点数（1..500） */
    volatile int32_t arg2;             /* 0x08 强制 Target_speed（0 = 不管） */
    volatile int32_t turn_kp_x1e6;     /* 0x0C Turn_Kp   x1e6 */
    volatile int32_t turn_kd_x1e6;     /* 0x10 Turn_Kd   x1e6 */
    volatile int32_t turn_max;         /* 0x14 Turn_Out_Max（固件硬上限 40） */
    volatile int32_t turn_sign;        /* 0x18 Turn_Sign（±1） */
    volatile int32_t vert_kp_x1e6;     /* 0x1C Vertical_Kp x1e6 */
    volatile int32_t vert_kd_x1e6;     /* 0x20 Vertical_Kd x1e6 */
    volatile int32_t vel_kp_x1e6;      /* 0x24 Velocity_Kp x1e6 */
    volatile int32_t vel_ki_x1e6;      /* 0x28 Velocity_Ki x1e6 */
    volatile int32_t step_amp;         /* 0x2C 阶跃幅度（±150） */
    volatile int32_t cmd;              /* 0x30 命令码，必须最后写 */

    /* ---------------- 目标 -> 主机（只读） ---------------- */
    volatile uint32_t ack;             /* 0x34 每消费一条命令 +1 */
    volatile uint32_t magic;           /* 0x38 固化后上电为 TUNE_MAGIC */
    volatile uint32_t state;           /* 0x3C TUNE_ST_* */
    volatile uint32_t capture_count;   /* 0x40 最近一次采集实际点数 */
    volatile uint32_t live_tick;       /* 0x44 中断计数（10ms 一拍，用于验活/验不复位） */
    volatile int32_t  isr_us_last;     /* 0x48 上一拍 Control() 耗时 us */
    volatile int32_t  isr_us_max;      /* 0x4C 窗口内最大耗时 us */
    volatile uint32_t err_count;       /* 0x50 未知命令/异常计数 */
    volatile int32_t  live_gyroz;      /* 0x54 */
    volatile int32_t  live_target_turn;/* 0x58 */
    volatile int32_t  live_turn_out;   /* 0x5C */
    volatile int32_t  live_moto1;      /* 0x60 */
    volatile int32_t  live_moto2;      /* 0x64 */
    volatile int32_t  live_roll_cdeg;  /* 0x68 0.01 度 */
    volatile int32_t  live_enc_l;      /* 0x6C */
    volatile int32_t  live_enc_r;      /* 0x70 */
    volatile int32_t  live_speed_tgt;  /* 0x74 */
    volatile int32_t  live_vert_out;   /* 0x78 */
    volatile int32_t  live_vel_out;    /* 0x7C */
    volatile uint32_t trial_count;     /* 0x80 已完成试次数 */
    volatile int32_t  applied_kp_x1e6; /* 0x84 当前实际生效的 Kp */
    volatile int32_t  applied_kd_x1e6; /* 0x88 */
    volatile int32_t  applied_max;     /* 0x8C */
    volatile int32_t  applied_sign;    /* 0x90 */
    int32_t  trial[TUNE_TRIAL_MAX][TUNE_TRIAL_COLS];   /* 0x94 */
    int16_t  buf[TUNE_BUF_SAMPLES][TUNE_BUF_CH];       /* 0x454 */
} tune_mailbox_t;

/* 接口契约的编译期断言：主机脚本按这些偏移寻址，不一致就编不过 */
typedef char tune_assert_cmd[(offsetof(tune_mailbox_t, cmd)   == 0x30) ? 1 : -1];
typedef char tune_assert_ack[(offsetof(tune_mailbox_t, ack)   == 0x34) ? 1 : -1];
typedef char tune_assert_trial[(offsetof(tune_mailbox_t, trial) == 0x94) ? 1 : -1];
typedef char tune_assert_buf[(offsetof(tune_mailbox_t, buf)   == 0x454) ? 1 : -1];

/* 试次行各列的含义（README 与 report 脚本共用） */
#define TUNE_TC_KP      0
#define TUNE_TC_KD      1
#define TUNE_TC_TMAX    2
#define TUNE_TC_SIGN    3
#define TUNE_TC_AMP     4
#define TUNE_TC_SS      5   /* 稳态偏航角速度（gyroz 原始值） */
#define TUNE_TC_RISE    6   /* 上升时间 ms，-1 表示无有效信号 */
#define TUNE_TC_OVER    7   /* 超调，千分比 */
#define TUNE_TC_OSC     8   /* 稳态段峰峰值，千分比 */
#define TUNE_TC_ROLL    9   /* |roll-Med| 峰值，0.01 度 */
#define TUNE_TC_SAT     10  /* 电机饱和占比，千分比 */
#define TUNE_TC_FLAGS   11

extern volatile tune_mailbox_t g_tune;

void Tune_Init(void);

/* 必须包住 Control()：Begin 在 Control() 前，End 在 Control() 后 */
void Tune_IsrBegin(void);
void Tune_IsrEnd(void);

#endif /* TUNE_H */
