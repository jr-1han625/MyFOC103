/*
 * FOC 公式算子：只进行数值计算，不访问 ADC、PWM 或业务全局数据。
 * 采用幅值不变 Clarke，q 轴超前 d 轴 90°；电流 A、电压 V、角度 rad。
 * 调用者提供同一采样时刻的 A/B 相电流与电角度，管理每轴 PI 历史状态。
 * 普通算子要求输入有限、指针有效；换算/限幅/SVPWM 返回 0 表示输入无效。
 * Park 与反 Park 传入对应时刻的 sin(theta_e)、cos(theta_e)。
 *
 * 学习资料：C:\Users\RD68\Desktop\FOC（下文“第 xx 课”均指此目录讲义）。
 * 阅读顺序：第 01 课坐标变换 -> 第 02 课电流闭环/SVPWM -> 第 08 课电压限幅。
 * 第 06 课说明电角度来源与时间戳，第 07 课说明前馈在电压合成处的含义。
 * 讲义符号映射：Ialpha/Ibeta -> alpha/beta，Id/Iq -> d/q；
 * Ud/Uq -> PI 输出 pi_v，Vd*、Vq* -> requested_v，Vd,lim/Vq,lim -> limited_v。
 * “*”表示目标值，不是乘号；坐标变换改变分量表示，不改变对应的物理矢量。
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

#define FOC_INV_SQRT3 0.5773502691896258f /* 1/sqrt(3)：Clarke 与 SVPWM 电压圆的系数。 */
#define FOC_SQRT3_OVER_2 0.8660254037844386f /* sqrt(3)/2：反 Clarke 的相轴投影系数。 */

/* 两电阻采样标度，由标定/配置提供；正电流约定为流入电机。 */
typedef struct {
    float offset_a_count; /* A 相零电流 ADC 码，可为小数。 */
    float offset_b_count; /* B 相零电流 ADC 码，可为小数。 */
    float amp_per_count_a; /* A/count，有符号，包含采样电阻、增益和电路极性。 */
    float amp_per_count_b; /* A/count，有符号，计算式为 Vref/(ADC满码*R*G)。 */
    uint16_t adc_max_count; /* ADC 满码，例如 12 bit 为 4095。 */
} FocCurrentCalibration;

/* 三相瞬时电流，算子输出；C 相由 ia+ib+ic=0 重构。 */
typedef struct {
    float a; /* A，相 A。 */
    float b; /* A，相 B。 */
    float c; /* A，相 C；两电阻重构值，不是独立测量。 */
} FocPhaseCurrent;

/* 第 01 课静止坐标：轴固定在定子上；调用位置决定单位：电流 A 或电压 V。 */
typedef struct {
    float alpha; /* alpha 轴，沿 A 相轴。 */
    float beta;  /* beta 轴，超前 alpha 轴 90°。 */
} FocAlphaBeta;

/* 第 01 课旋转坐标：d 轴对齐转子永磁磁链，q 轴超前 90°，单位为 A 或 V。 */
typedef struct {
    float d; /* d 轴，沿电角度指向。 */
    float q; /* q 轴，超前 d 轴 90°。 */
} FocDq;

/* 每轴 PI 参数，由调用者在停机时配置。 */
typedef struct {
    float kp; /* V/A，比例系数，非负。 */
    float ki; /* V/(A*s)，连续域积分系数，非负；算子内部乘 Ts。 */
} FocPiGains;

/* 每轴 PI 的唯一历史状态，由算子及整链的抗饱和处理更新。 */
typedef struct {
    float integral_v; /* V，积分输出；每个有效控制周期更新，停机/失败清零。 */
} FocPiState;

/* 扇区 SVPWM 算法结果，由算子一次写入；调用者读取，不直接写定时器。 */
typedef struct {
    uint8_t sector; /* 1~6：从 alpha 正轴起逆时针的 60° 扇区；0：零矢量或失败。 */
    float t1_ratio; /* [0,1]，T1/Tpwm，本扇区低角度有效矢量的周期作用比例。 */
    float t2_ratio; /* [0,1]，T2/Tpwm，本扇区高角度有效矢量的周期作用比例。 */
    float t0_ratio; /* [0,1]，T0/Tpwm，全部零矢量时间比例；三者之和为 1。 */
    float duty_a; /* [0,1]，A 相上桥臂有效占空比，硬件极性由驱动适配。 */
    float duty_b; /* [0,1]，B 相上桥臂有效占空比。 */
    float duty_c; /* [0,1]，C 相上桥臂有效占空比。 */
} FocSvpwm;

/* 原始 ADC 码扣零点后换算 A/B 相电流，再重构 C 相；失败输出清零。 */
int Foc_CurrentSample(uint16_t raw_a, uint16_t raw_b,
                      const FocCurrentCalibration *cal, FocPhaseCurrent *current);
FocAlphaBeta Foc_Clarke(FocPhaseCurrent current);
FocDq Foc_Park(FocAlphaBeta current, float sin_theta, float cos_theta);
/* 每周期恰好调用一次：先积分，再输出 Kp*error + integral，Ts 单位 s。 */
float Foc_PiStep(float error_a, float period_s, FocPiGains gains, FocPiState *state);
/* PI 与外部给定前馈相加，按圆形边界等比例限幅；scale 输出 [0,1]。 */
int Foc_VoltageLimit(FocDq pi_v, FocDq feedforward_v, float limit_v,
                     FocDq *requested_v, FocDq *limited_v, float *scale);
FocAlphaBeta Foc_InversePark(FocDq voltage, float sin_theta, float cos_theta);
/* 纯算法：判断扇区、计算 T1/T2/T0 比例并对称分配占空比，不访问硬件或业务状态。
 * 输入须在母线六边形可实现范围内；失败返回 0，扇区/有效时间清零，占空比置 0.5。
 * Tpwm 为实际 PWM 周期，不在此指定；时间秒值由调用者用比例乘 Tpwm 得到。
 */
int Foc_Svpwm(FocAlphaBeta voltage, float bus_v, FocSvpwm *pwm);

#endif
