/*
 * FOC 基本电流环：接收一次同步采样，依次执行公式算子，输出三相占空比。
 * foc 是本模块唯一公开对象；停机时配置标度、PI、周期和电压利用率，再使能。
 * Foc_ControlStep 由采样完成后的控制入口每周期调用一次；本模块不配置外设。
 * 当前实机启动走 motor_start 的电压开环路径；本电流 PI 链仍独立验收，valid 仅指数值成功。
 * 配置/命令与控制计算不得并发写入；前台改参须在控制中断停用后完成。
 *
 * 学习对照：第 01 课看 current_a/current_ab_a/current_dq_a；
 * 第 02 课看 error_a/pi_v/voltage_ab_v/pwm；第 08 课看 requested_v/limited_v。
 * 第 07 课前馈目前仅由 feedforward_v 接收外部结果，未在此实现电机模型计算。
 */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_math.h"

/* 一次控制周期的完整输入快照，由采样/调度入口提供。 */
typedef struct {
    uint16_t raw_a; /* A 相 ADC 原始码，与 raw_b 来自同一有效 PWM 采样窗口。 */
    uint16_t raw_b; /* B 相 ADC 原始码。 */
    float bus_v; /* V，本周期母线电压，须大于 0。 */
    float sample_angle_rad; /* rad，采样时刻电角度，供 Park 使用；对应第 06 课 theta_sample。 */
    float output_angle_rad; /* rad，电压生效时刻电角度，供反 Park 使用；对应第 06 课 theta_act。 */
} FocSample;

/* 控制结果状态；任何非 OK 状态均禁止调用者把占空比作为有效驱动输出。 */
typedef enum {
    FOC_DISABLED = 0,
    FOC_OK,
    FOC_INVALID_CONFIG,
    FOC_INVALID_SAMPLE,
    FOC_NUMERIC_ERROR
} FocStatus;

/* 电流环模块的唯一公开数据，输入由调用者写，状态和中间结果由本模块写。 */
typedef struct {
    FocCurrentCalibration calibration; /* 输入：采样标度，停机配置。 */
    FocPiGains gains_d; /* 输入：d 轴 PI，停机配置。 */
    FocPiGains gains_q; /* 输入：q 轴 PI，停机配置。 */
    float period_s; /* 输入：实际调用周期 s；不是主循环延时或估计周期。 */
    float voltage_utilization; /* 输入：(0,1]，限幅半径为此值*Vdc/sqrt(3)。 */
    uint8_t enabled; /* 输入：0 停机并清除积分，1 执行一次电流环。 */
    FocDq reference_a; /* 输入：Id*、Iq*，单位 A，由调用者在控制调用前给定。 */
    FocDq feedforward_v; /* 输入：外部给定的 Vd/Vq 前馈 V；基础电流环设为 0。 */
    FocPiState pi_d; /* 状态：d 轴积分，模块更新；外部只读。 */
    FocPiState pi_q; /* 状态：q 轴积分，模块更新；外部只读。 */
    FocPhaseCurrent current_a; /* 输出：三相采样电流 A。 */
    FocAlphaBeta current_ab_a; /* 输出：Clarke 电流 A。 */
    FocDq current_dq_a; /* 输出：Park 电流 A。 */
    FocDq error_a; /* 输出：电流目标减反馈 A。 */
    FocDq pi_v; /* 输出：第 02/07 课 Ud/Uq，本周期回算校正前的 PI 电压 V。 */
    FocDq requested_v; /* 输出：第 07/08 课带星号的目标 Vd、Vq，PI + 前馈电压 V。 */
    FocDq limited_v; /* 输出：第 08 课 Vd,lim/Vq,lim，圆形限幅后电压 V。 */
    float voltage_limit_v; /* 输出：本周期可用电压半径 V。 */
    float voltage_scale; /* 输出：限幅缩放比例 [0,1]；1 表示未限幅。 */
    FocAlphaBeta voltage_ab_v; /* 输出：反 Park 的 alpha/beta 电压 V。 */
    FocSvpwm pwm; /* 输出：SVPWM 扇区、作用时间比例和占空比；valid=1 时才可应用。 */
    FocStatus status; /* 输出：当前调用结果，外部只读。 */
    uint8_t valid; /* 输出：1 表示本次计算完整成功；不能替代硬件保护许可。 */
} FocControl;

extern FocControl foc;

/* 清零配置、命令和积分，保持未使能；必须在任何控制中断启动前调用。 */
void Foc_Init(void);
/* sample 指向本周期只读快照且不可为空；返回状态并更新 foc 的逐级结果。 */
FocStatus Foc_ControlStep(const FocSample *sample);

#endif
