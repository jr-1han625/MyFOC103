#include "foc_control.h"
#include <math.h>
#include <string.h>

FocControl foc;

/* 初始化唯一对象；未配置硬件标度/周期/增益前保持停机，占空比只作中性显示。 */
void Foc_Init(void)
{
    memset(&foc, 0, sizeof(foc));
    foc.pwm.t0_ratio = 1.0f;
    foc.pwm.duty_a = 0.5f;
    foc.pwm.duty_b = 0.5f;
    foc.pwm.duty_c = 0.5f;
    foc.status = FOC_DISABLED;
}

/*
 * 消费一次采样快照，按总框图顺序计算；失败清积分和本周期输出，返回失败原因。
 * 学习阅读顺序：第 01 课（三相 -> alpha/beta -> d/q），第 02 课（电流 PI -> PWM），
 * 第 07/08 课（PI 与前馈合成 -> 电压矢量限幅 -> anti-windup）。
 * sample 的电流原始码、母线和电角度由调用者提供，不在本函数生成开环角度。
 */
FocStatus Foc_ControlStep(const FocSample *sample)
{
    float sin_sample;
    float cos_sample;
    float sin_output;
    float cos_output;

    foc.valid = 0;
    if (foc.enabled == 0) {
        foc.status = FOC_DISABLED;
        goto inactive;
    }
    if (foc.enabled != 1 || !isfinite(foc.period_s) || foc.period_s <= 0.0f ||
        !isfinite(foc.voltage_utilization) || foc.voltage_utilization <= 0.0f ||
        foc.voltage_utilization > 1.0f ||
        !isfinite(foc.gains_d.kp) || foc.gains_d.kp < 0.0f ||
        !isfinite(foc.gains_d.ki) || foc.gains_d.ki < 0.0f ||
        !isfinite(foc.gains_q.kp) || foc.gains_q.kp < 0.0f ||
        !isfinite(foc.gains_q.ki) || foc.gains_q.ki < 0.0f ||
        !isfinite(foc.reference_a.d) || !isfinite(foc.reference_a.q) ||
        !isfinite(foc.feedforward_v.d) || !isfinite(foc.feedforward_v.q)) {
        foc.status = FOC_INVALID_CONFIG;
        goto inactive;
    }
    if (!isfinite(sample->bus_v) || sample->bus_v <= 0.0f ||
        !isfinite(sample->sample_angle_rad) || !isfinite(sample->output_angle_rad)) {
        foc.status = FOC_INVALID_SAMPLE;
        goto inactive;
    }

    /* 1. 总框图 ADC 环节：同一 PWM 周期原始码 -> 扣零点、换算 A、重构 Ic。
     * current_a 是第 01 课的 Ia/Ib/Ic；Ic=-Ia-Ib 使用无零序三相条件。
     */
    if (!Foc_CurrentSample(sample->raw_a, sample->raw_b,
                           &foc.calibration, &foc.current_a)) {
        foc.status = FOC_INVALID_SAMPLE;
        goto inactive;
    }

    /* 2. 第 01 课 Clarke：Ialpha=Ia，Ibeta=(Ia+2Ib)/sqrt(3)。
     * current_ab_a 是定子静止轴上的同一电流矢量，不是额外采样的两路电流。
     */
    foc.current_ab_a = Foc_Clarke(foc.current_a);

    /* 3. 第 01 课 Park：采样时刻电角度将静止电流投影到转子磁链坐标。
     * 第 06 课给出有感角度关系 theta_e=wrap(s*p*theta_enc+theta_offset)。
     * 这里接收的已经是电角度 rad；角度来源、极对数和零位换算不在本函数实现。
     */
    sin_sample = sinf(sample->sample_angle_rad);
    cos_sample = cosf(sample->sample_angle_rad);
    foc.current_dq_a = Foc_Park(foc.current_ab_a, sin_sample, cos_sample);

    /* 4. 第 02 课双轴电流 PI：ed=Id*-Id，eq=Iq*-Iq，U=Kp*e+Ki*积分(e dt)。
     * reference_a 对应讲义带星号的电流目标，pi_v 对应纠偏电压 Ud/Uq，而非占空比。
     */
    foc.error_a.d = foc.reference_a.d - foc.current_dq_a.d;
    foc.error_a.q = foc.reference_a.q - foc.current_dq_a.q;
    foc.pi_v.d = Foc_PiStep(foc.error_a.d, foc.period_s, foc.gains_d, &foc.pi_d);
    foc.pi_v.q = Foc_PiStep(foc.error_a.q, foc.period_s, foc.gains_q, &foc.pi_q);

    /* 5. 第 07 课电压合成：Vd*=Ud+Vd_ff，Vq*=Uq+Vq_ff，基础链前馈填零。
     * 讲义旋转解耦/反电动势前馈为 Vd_ff=-omega_e*Lq*Iq，
     * Vq_ff=omega_e*(Ld*Id+psi_f)；这里只接收 feedforward_v，不计算这些模型项。
     * 第 08 课电压限幅：六个有效矢量幅值为 2*Vdc/3，六边形内切圆半径为
     * Vdc/sqrt(3)；乘 voltage_utilization 后预留调制余量，单位仍是电压 V。
     * 它限制 sqrt(Vd²+Vq²)，不是电流限幅，也不会自动产生弱磁负 Id*。
     */
    foc.voltage_limit_v = sample->bus_v * FOC_INV_SQRT3 * foc.voltage_utilization;
    if (!Foc_VoltageLimit(foc.pi_v, foc.feedforward_v, foc.voltage_limit_v,
                          &foc.requested_v, &foc.limited_v, &foc.voltage_scale)) {
        foc.status = FOC_NUMERIC_ERROR;
        goto inactive;
    }

    /* 第 08 课要求限幅配合 anti-windup；此处具体采用单位增益离散跟踪回算：
     * Ui_next=Ui_trial+(Vlimited-Vrequested)，Ui 是积分电压，不是相电流 I。
     * 电压受限后不继续按“未受限请求可被实现”累积积分；修正作用于下一周期。
     * 两轴跟踪矢量限幅结果，包含前馈影响；未限幅时修正量为零。
     * 这条具体回算公式是当前工程实现选择，讲义没有指定它为唯一 anti-windup 算法。
     */
    foc.pi_d.integral_v += foc.limited_v.d - foc.requested_v.d;
    foc.pi_q.integral_v += foc.limited_v.q - foc.requested_v.q;
    if (!isfinite(foc.pi_d.integral_v) || !isfinite(foc.pi_q.integral_v)) {
        foc.status = FOC_NUMERIC_ERROR;
        goto inactive;
    }

    /* 6. 第 02 课反 Park：limited_v -> 定子静止电压 voltage_ab_v。
     * 第 06 课预测关系 theta_act 约等于 wrap(theta_sample+omega_e*延迟时间)，
     * 此处不计算预测，只使用调用者传入的 output_angle_rad；无补偿时两角相同。
     */
    sin_output = sinf(sample->output_angle_rad);
    cos_output = cosf(sample->output_angle_rad);
    foc.voltage_ab_v = Foc_InversePark(foc.limited_v, sin_output, cos_output);

    /* 7. 第 02 课 SVPWM：扇区判断 -> T1/T2/T0 周期比例 -> Da/Db/Dc。
     * 结果是一个 PWM 周期内的目标平均电压占空比；真实绕组电流随后由电机动态决定，
     * 并不会在赋值瞬间跳到目标 Id_ref/Iq_ref。
     */
    if (!Foc_Svpwm(foc.voltage_ab_v, sample->bus_v, &foc.pwm)) {
        foc.status = FOC_NUMERIC_ERROR;
        goto inactive;
    }
    foc.status = FOC_OK;
    foc.valid = 1;
    return foc.status;

inactive:
    foc.pi_d.integral_v = 0.0f;
    foc.pi_q.integral_v = 0.0f;
    foc.current_a = (FocPhaseCurrent){0.0f, 0.0f, 0.0f};
    foc.current_ab_a = (FocAlphaBeta){0.0f, 0.0f};
    foc.current_dq_a = (FocDq){0.0f, 0.0f};
    foc.error_a = (FocDq){0.0f, 0.0f};
    foc.pi_v = (FocDq){0.0f, 0.0f};
    foc.requested_v = (FocDq){0.0f, 0.0f};
    foc.limited_v = (FocDq){0.0f, 0.0f};
    foc.voltage_limit_v = 0.0f;
    foc.voltage_scale = 0.0f;
    foc.voltage_ab_v = (FocAlphaBeta){0.0f, 0.0f};
    foc.pwm = (FocSvpwm){0, 0.0f, 0.0f, 1.0f, 0.5f, 0.5f, 0.5f};
    return foc.status;
}
