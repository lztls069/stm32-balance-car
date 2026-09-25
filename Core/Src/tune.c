/**
 * @file    tune.c
 * @brief   SWD 在线调参：mailbox + 片上抓波形 + 试次指标。
 *
 * 时序（Control() 跑在 MPU6050 的 EXTI 中断里，约 100Hz）：
 *   Tune_IsrBegin();  Control();  Tune_IsrEnd();
 * End 里做三件事：测 Control() 耗时、消费 mailbox 命令、推进采集状态机。
 * 指标计算只在一次采集结束时做一遍，其余每拍都是常数开销。
 *
 * 采集期间 tune_manual = 1，Control() 不再被遥控按键改写 Target_turn /
 * Target_speed，由本模块按「延时 -> 阶跃保持 -> 归零 -> 恢复」推进。
 */
#include "main.h"
#include "tune.h"
#include "pid.h"
#include <math.h>
#include <string.h>

/* 固件侧硬上限：主机写超了也按这里的值执行 */
#define TUNE_TMAX_HARD   40
#define TUNE_AMP_HARD    150
#define TUNE_FELL_DEG    45.0f
#define TUNE_DIVERGE_RAW 8000   /* ≈488 dps，正常实验不可能到这 */
#define TUNE_MIN_SS_RAW  50     /* 稳态角速度低于此值认为没信号 */

volatile tune_mailbox_t g_tune;

static uint32_t s_t0;            /* Control() 入口的 DWT 周期数 */
static uint32_t s_cyc_per_us;    /* 72MHz -> 72 */
static uint16_t s_idx;           /* 当前采集点序号 */
static uint16_t s_len;           /* 本次采集点数 */
static uint16_t s_delay;         /* 采样前等待的拍数 */
static uint16_t s_step_at;       /* 阶跃加入的序号 */
static uint16_t s_release_at;    /* 阶跃撤销的序号 */
static int16_t  s_step_amp;
static int16_t  s_cmd_now;       /* 斜坡发生器当前目标 */
static int16_t  s_ramp_step;     /* 每拍变化量，0 = 硬阶跃 */
static uint8_t  s_armed;
static uint8_t  s_fell;

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------------------------------------------------------------------------
 * 参数应用
 * ------------------------------------------------------------------------- */

static void tune_apply(void)
{
    int32_t m = clamp_i32(g_tune.turn_max, 0, TUNE_TMAX_HARD);
    int32_t s = (g_tune.turn_sign < 0) ? -1 : 1;

    Turn_Kp = (float)g_tune.turn_kp_x1e6 / 1000000.0f;
    Turn_Kd = (float)g_tune.turn_kd_x1e6 / 1000000.0f;
    Turn_Out_Max = (int)m;
    Turn_Sign = (int)s;

    Vertical_Kp = (float)g_tune.vert_kp_x1e6 / 1000000.0f;
    Vertical_Kd = (float)g_tune.vert_kd_x1e6 / 1000000.0f;
    Velocity_Kp = (float)g_tune.vel_kp_x1e6 / 1000000.0f;
    Velocity_Ki = (float)g_tune.vel_ki_x1e6 / 1000000.0f;

    /* 回写钳位后的真实生效值 */
    g_tune.turn_max  = m;
    g_tune.turn_sign = s;
    g_tune.applied_kp_x1e6 = g_tune.turn_kp_x1e6;
    g_tune.applied_kd_x1e6 = g_tune.turn_kd_x1e6;
    g_tune.applied_max      = m;
    g_tune.applied_sign     = s;
}

/* ---------------------------------------------------------------------------
 * 采集
 * ------------------------------------------------------------------------- */

static void tune_push(void)
{
    g_tune.buf[s_idx][TUNE_CH_GYROZ] = gyroz;
    g_tune.buf[s_idx][TUNE_CH_TTURN] = (int16_t)Target_turn;
    g_tune.buf[s_idx][TUNE_CH_TOUT]  = (int16_t)Turn_out;
    g_tune.buf[s_idx][TUNE_CH_MOTO1] = (int16_t)MOTO1;
    g_tune.buf[s_idx][TUNE_CH_MOTO2] = (int16_t)MOTO2;
    g_tune.buf[s_idx][TUNE_CH_ROLL]  = (int16_t)(roll * 100.0f);
    g_tune.buf[s_idx][TUNE_CH_ENCL]  = (int16_t)Encoder_Left;
    g_tune.buf[s_idx][TUNE_CH_ENCR]  = (int16_t)Encoder_Right;
    s_idx++;
}

static void tune_start(uint16_t len, uint16_t delay_ms, int16_t amp, int16_t ramp_ticks)
{
    if (len == 0u) len = 1u;
    if (len > TUNE_BUF_SAMPLES) len = (uint16_t)TUNE_BUF_SAMPLES;

    amp = (int16_t)clamp_i32(amp, -TUNE_AMP_HARD, TUNE_AMP_HARD);
    ramp_ticks = (int16_t)clamp_i32(ramp_ticks, 0, 1000);

    g_tune.arg1     = TUNE_ARG1(len, ramp_ticks);
    g_tune.step_amp = (int32_t)amp;

    s_len        = len;
    s_delay      = (uint16_t)(clamp_i32((int32_t)delay_ms, 0, 10000) / 10);
    s_step_amp   = amp;
    s_ramp_step  = (ramp_ticks > 0)
                 ? (int16_t)clamp_i32(iabs32(amp) / (int32_t)ramp_ticks + 1, 1, TUNE_AMP_HARD)
                 : 0;
    s_cmd_now    = 0;
    s_step_at    = (uint16_t)(len / 5u);            /* 前 20% 基线 */
    s_release_at = (uint16_t)(len - len / 5u);      /* 后 20% 恢复 */
    if (s_release_at <= s_step_at) {
        s_step_at    = (uint16_t)(len / 2u);
        s_release_at = len;
    }
    s_idx   = 0u;
    s_fell  = 0u;
    s_armed = 1u;

    tune_manual  = 1u;
    Target_turn  = 0;
    Target_speed = (int)g_tune.arg2;

    g_tune.state &= ~(TUNE_ST_FULL | TUNE_ST_ARMED | TUNE_ST_FELL);
    g_tune.state |= (TUNE_ST_ARMED | TUNE_ST_MANUAL);
}

/* 一次采集结束后在片内算指标，只留一行试次表；原始波形留给主机 dump */
static void tune_finish(void)
{
    uint16_t i, n = 0, row;
    uint16_t w0 = s_step_at, w1 = s_release_at, s0;
    int32_t sum = 0, ss, peak = 0, mn = 32767, mx = -32768;
    int32_t roll_peak = 0, med_cdeg;
    int32_t rise = -1, over = 0, osc = 0, sat_permille = 0, flags = 0;
    uint16_t sat = 0;

    if (w1 <= w0) { w0 = 0u; w1 = s_len; }
    s0 = (uint16_t)(w1 - (uint16_t)(((uint32_t)(w1 - w0) * 3u) / 10u));
    if (s0 <= w0) s0 = w0;

    for (i = s0; i < w1; i++) sum += (int32_t)g_tune.buf[i][TUNE_CH_GYROZ];
    ss = (w1 > s0) ? (sum / (int32_t)(w1 - s0)) : 0;

    med_cdeg = (int32_t)(Med_Angle * 100.0f);

    for (i = w0; i < w1; i++) {
        int32_t g  = (int32_t)g_tune.buf[i][TUNE_CH_GYROZ];
        int32_t ag = iabs32(g);
        int32_t r  = iabs32((int32_t)g_tune.buf[i][TUNE_CH_ROLL] - med_cdeg);
        int32_t m1 = iabs32((int32_t)g_tune.buf[i][TUNE_CH_MOTO1]);
        int32_t m2 = iabs32((int32_t)g_tune.buf[i][TUNE_CH_MOTO2]);

        if (ag > peak) peak = ag;
        if (g < mn) mn = g;
        if (g > mx) mx = g;
        if (r > roll_peak) roll_peak = r;
        if ((m1 >= 99) || (m2 >= 99)) sat++;
        n++;
    }

    if (iabs32(ss) >= TUNE_MIN_SS_RAW) {
        int32_t thr = (iabs32(ss) * 9) / 10;
        for (i = w0; i < w1; i++) {
            if (iabs32((int32_t)g_tune.buf[i][TUNE_CH_GYROZ]) >= thr) {
                rise = (int32_t)(i - w0) * 10;
                break;
            }
        }
        over = ((peak - iabs32(ss)) * 1000) / iabs32(ss);
        if (over < 0) over = 0;
        osc = ((mx - mn) * 1000) / iabs32(ss);
    } else {
        flags |= TUNE_TR_NOSIGNAL;
    }

    if (peak > TUNE_DIVERGE_RAW) flags |= TUNE_TR_DIVERGE;
    if (s_fell)                  flags |= TUNE_TR_FELL;
    if (s_step_amp < 0)          flags |= TUNE_TR_NEGSTEP;
    if (n) sat_permille = ((int32_t)sat * 1000) / (int32_t)n;

    row = (uint16_t)(g_tune.trial_count % TUNE_TRIAL_MAX);
    g_tune.trial[row][TUNE_TC_KP]    = g_tune.applied_kp_x1e6;
    g_tune.trial[row][TUNE_TC_KD]    = g_tune.applied_kd_x1e6;
    g_tune.trial[row][TUNE_TC_TMAX]  = g_tune.applied_max;
    g_tune.trial[row][TUNE_TC_SIGN]  = g_tune.applied_sign;
    g_tune.trial[row][TUNE_TC_AMP]   = (int32_t)s_step_amp;
    g_tune.trial[row][TUNE_TC_SS]    = ss;
    g_tune.trial[row][TUNE_TC_RISE]  = rise;
    g_tune.trial[row][TUNE_TC_OVER]  = over;
    g_tune.trial[row][TUNE_TC_OSC]   = osc;
    g_tune.trial[row][TUNE_TC_ROLL]  = roll_peak;
    g_tune.trial[row][TUNE_TC_SAT]   = sat_permille;
    g_tune.trial[row][TUNE_TC_FLAGS] = flags;
    g_tune.trial_count++;
}

/* ---------------------------------------------------------------------------
 * 命令
 * ------------------------------------------------------------------------- */

static void tune_handle(int32_t c)
{
    switch (c) {
    case TUNE_CMD_APPLY:
        tune_apply();
        break;

    case TUNE_CMD_ARM:
        tune_apply();
        tune_start((uint16_t)TUNE_ARG1_SAMPLES(g_tune.arg1), (uint16_t)g_tune.arg0, 0, 0);
        break;

    case TUNE_CMD_STEP:
        tune_apply();
        tune_start((uint16_t)TUNE_ARG1_SAMPLES(g_tune.arg1), (uint16_t)g_tune.arg0,
                   (int16_t)g_tune.step_amp, (int16_t)TUNE_ARG1_RAMP(g_tune.arg1));
        break;

    case TUNE_CMD_STOP:
        motor_enable = 0u;
        tune_manual  = 0u;
        s_armed      = 0u;
        Target_turn  = 0;
        Target_speed = 0;
        g_tune.state &= ~(TUNE_ST_ARMED | TUNE_ST_MANUAL);
        break;

    case TUNE_CMD_CLRTRIAL:
        for (uint16_t r = 0u; r < TUNE_TRIAL_MAX; r++) {
            for (uint16_t k = 0u; k < TUNE_TRIAL_COLS; k++) {
                g_tune.trial[r][k] = 0;
            }
        }
        g_tune.trial_count = 0u;
        break;

    case TUNE_CMD_CLRSTAT:
        g_tune.isr_us_max = 0;
        g_tune.err_count  = 0u;
        break;

    default:
        g_tune.err_count++;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * 中断侧入口
 * ------------------------------------------------------------------------- */

void Tune_IsrBegin(void)
{
    s_t0 = DWT->CYCCNT;
}

void Tune_IsrEnd(void)
{
    uint32_t us = (DWT->CYCCNT - s_t0) / s_cyc_per_us;

    g_tune.isr_us_last = (int32_t)us;
    if ((int32_t)us > g_tune.isr_us_max) g_tune.isr_us_max = (int32_t)us;

    g_tune.live_tick++;

    if (g_tune.cmd != TUNE_CMD_IDLE) {
        int32_t c = g_tune.cmd;
        g_tune.cmd = TUNE_CMD_IDLE;     /* 目标消费掉命令，同一条命令可重复下发 */
        tune_handle(c);
        g_tune.ack++;
    }

    if (s_armed) {
        if (fabsf(roll - Med_Angle) > TUNE_FELL_DEG) s_fell = 1u;

        if (s_delay) {
            s_delay--;
        } else {
            if (s_idx == s_step_at) {
                s_cmd_now = s_step_amp;
            } else if (s_idx == s_release_at) {
                s_cmd_now = 0;
            }
            /* 斜坡模式每拍只走一小步，避免硬阶跃激起平衡环极限环 */
            if (s_ramp_step > 0) {
                int32_t diff = (int32_t)s_cmd_now - (int32_t)Target_turn;
                if (diff > s_ramp_step)       Target_turn += s_ramp_step;
                else if (diff < -s_ramp_step) Target_turn -= s_ramp_step;
                else                          Target_turn = s_cmd_now;
            } else {
                Target_turn = s_cmd_now;
            }
            tune_push();

            if (s_idx >= s_len) {
                s_armed = 0u;
                g_tune.capture_count = s_idx;
                g_tune.state &= ~(TUNE_ST_ARMED | TUNE_ST_MANUAL);
                g_tune.state |= TUNE_ST_FULL;
                if (s_fell) g_tune.state |= TUNE_ST_FELL;
                tune_finish();
                tune_manual  = 0u;      /* 交还遥控按键 */
                Target_turn  = 0;
                Target_speed = 0;
            }
        }
    }

    /* 单次读取就能看到车当前状态 */
    g_tune.live_gyroz        = gyroz;
    g_tune.live_target_turn  = Target_turn;
    g_tune.live_turn_out     = Turn_out;
    g_tune.live_moto1        = MOTO1;
    g_tune.live_moto2        = MOTO2;
    g_tune.live_roll_cdeg    = (int32_t)(roll * 100.0f);
    g_tune.live_enc_l        = Encoder_Left;
    g_tune.live_enc_r        = Encoder_Right;
    g_tune.live_speed_tgt    = Target_speed;
    g_tune.live_vert_out     = Vertical_out;
    g_tune.live_vel_out      = Velocity_out;
}

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */

void Tune_Init(void)
{
    uint32_t div = SystemCoreClock / 1000000u;

    s_cyc_per_us = (div == 0u) ? 1u : div;

    memset((void *)&g_tune, 0, sizeof(g_tune));

    /* 默认值直接取 pid.c 里编译进去的初值，只读一遍 mailbox 就知道当前生效什么 */
    g_tune.arg0 = 500;
    g_tune.arg1 = TUNE_ARG1(200, 0);
    g_tune.arg2 = 0;
    g_tune.turn_kp_x1e6 = (int32_t)(Turn_Kp * 1000000.0f);
    g_tune.turn_kd_x1e6 = (int32_t)(Turn_Kd * 1000000.0f);
    g_tune.turn_max      = Turn_Out_Max;
    g_tune.turn_sign     = Turn_Sign;
    g_tune.vert_kp_x1e6 = (int32_t)(Vertical_Kp * 1000000.0f);
    g_tune.vert_kd_x1e6 = (int32_t)(Vertical_Kd * 1000000.0f);
    g_tune.vel_kp_x1e6  = (int32_t)(Velocity_Kp * 1000000.0f);
    g_tune.vel_ki_x1e6  = (int32_t)(Velocity_Ki * 1000000.0f);
    g_tune.step_amp      = 40;
    g_tune.cmd           = TUNE_CMD_IDLE;

    g_tune.applied_kp_x1e6 = g_tune.turn_kp_x1e6;
    g_tune.applied_kd_x1e6 = g_tune.turn_kd_x1e6;
    g_tune.applied_max      = Turn_Out_Max;
    g_tune.applied_sign     = Turn_Sign;

    g_tune.ack           = 0u;
    g_tune.state         = 0u;
    g_tune.capture_count = 0u;
    g_tune.live_tick     = 0u;
    g_tune.trial_count   = 0u;
    g_tune.err_count     = 0u;

    s_armed = 0u;
    s_idx   = 0u;
    s_delay = 0u;
    tune_manual = 0u;

    /* DWT 周期计数器用于测 Control() 耗时 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_tune.magic = TUNE_MAGIC;

    /* Control() 由 MPU6050 INT 引脚(PB5)触发，必须使能才能跑起来 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
