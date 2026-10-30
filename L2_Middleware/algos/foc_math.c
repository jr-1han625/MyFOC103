#include "foc_math.h"
#include <math.h>

/*
 * 电流采样换算，对应总框图 ADC -> Clarke 之间的数据准备。
 * raw 为同一采样周期的原始码；cal 只读，current 输出 A，成功返回 1。
 * 零电流也可能对应非零 ADC 码，所以先扣 offset，再乘 A/count 标度。
 * 第 01 课“三相相轴与平衡条件”：三线制、无零序时 Ia+Ib+Ic=0。
 * 本算子据此重构 Ic，不代表实际测量了第三相，也不执行 ADC 外设采集。
 */
int Foc_CurrentSample(uint16_t raw_a, uint16_t raw_b,
                      const FocCurrentCalibration *cal, FocPhaseCurrent *current)
{
    *current = (FocPhaseCurrent){0.0f, 0.0f, 0.0f};
    if (cal->adc_max_count == 0 || raw_a > cal->adc_max_count ||
        raw_b > cal->adc_max_count ||
        !isfinite(cal->offset_a_count) || !isfinite(cal->offset_b_count) ||
        cal->offset_a_count < 0.0f || cal->offset_a_count > cal->adc_max_count ||
        cal->offset_b_count < 0.0f || cal->offset_b_count > cal->adc_max_count ||
        !isfinite(cal->amp_per_count_a) || !isfinite(cal->amp_per_count_b) ||
        cal->amp_per_count_a == 0.0f || cal->amp_per_count_b == 0.0f) {
        return 0;
    }

    /* I = (ADC - offset) * Vref / (ADC满码 * Rshunt * Gain)。 */
    current->a = ((float)raw_a - cal->offset_a_count) * cal->amp_per_count_a;
    current->b = ((float)raw_b - cal->offset_b_count) * cal->amp_per_count_b;
    current->c = -current->a - current->b;
    if (!isfinite(current->a) || !isfinite(current->b) || !isfinite(current->c)) {
        *current = (FocPhaseCurrent){0.0f, 0.0f, 0.0f};
        return 0;
    }
    return 1;
}

/*
 * 第 01 课“四、Clarke 变换”：三相电流 A -> 定子静止 alpha/beta 电流 A。
 * 幅值不变形式：Ialpha=(2/3)*(Ia-Ib/2-Ic/2)，Ibeta=(Ib-Ic)/sqrt(3)。
 * 代入 Ic=-Ia-Ib，得到下面的二电流形式；alpha 与 A 相轴重合。
 * “幅值不变”指平衡正弦三相的相电流峰值等于 alpha/beta 矢量幅值，
 * 不是把三相瞬时数值原样复制；这里只改变坐标表示，不改变真实绕组电流。
 * 讲义例：Ia=8.660 A、Ib=0 A -> Ialpha=8.660 A、Ibeta 约 5 A。
 */
FocAlphaBeta Foc_Clarke(FocPhaseCurrent current)
{
    FocAlphaBeta result;
    result.alpha = current.a;
    result.beta = (current.a + 2.0f * current.b) * FOC_INV_SQRT3;
    return result;
}

/*
 * 第 01 课“五、Park 变换”：把静止电流投影到随转子磁链旋转的 d/q 轴。
 * [Id] = [ cos(theta_e)  sin(theta_e)] [Ialpha]
 * [Iq]   [-sin(theta_e)  cos(theta_e)] [Ibeta ]
 * d 轴单位方向为 (cos,sin)，q 轴超前 90°，单位方向为 (-sin,cos)。
 * 因而 d/q 是同一电流矢量的两个投影，不是另外两根物理绕组的电流。
 * sin/cos 对应采样时刻的电角度，不能直接把机械角或角度制数值当作输入。
 * 讲义例：Ialpha=8.660、Ibeta 约 5，theta_e=300° -> Id 约 0、Iq 约 10 A。
 */
FocDq Foc_Park(FocAlphaBeta current, float sin_theta, float cos_theta)
{
    FocDq result;
    result.d = current.alpha * cos_theta + current.beta * sin_theta;
    result.q = -current.alpha * sin_theta + current.beta * cos_theta;
    return result;
}

/*
 * 第 02 课“三、dq 电流 PI”：电流误差 A -> 用于纠偏的电压指令 V。
 * 连续式 U=Kp*e+Ki*积分(e dt)，下面按每个采样周期进行矩形离散积分：
 * Ui_trial[k]=Ui[k-1]+Ki*e[k]*Ts，U[k]=Kp*e[k]+Ui_trial[k]。
 * error_a 是目标减反馈；period_s 为 Ts（秒），ki 尚未预乘 Ts，不能重复乘。
 * state 保存本轴唯一积分电压历史，每个有效周期更新一次；输出不是电流或占空比。
 * 两轴合成限幅之后，整链还会回算修正积分，所以本函数不独自做轴向电压截断。
 */
float Foc_PiStep(float error_a, float period_s, FocPiGains gains, FocPiState *state)
{
    state->integral_v += gains.ki * error_a * period_s;
    return gains.kp * error_a + state->integral_v;
}

/*
 * 第 07 课“四、模型解耦”与第 08 课“7. 电压限幅怎样进入 MCU 控制链”。
 * 先合成：Vd*=Ud+Vd_ff，Vq*=Uq+Vq_ff；pi_v 对应 Ud/Uq，前馈由调用者提供。
 * 再限幅：|V*|=sqrt(Vd*²+Vq*²)，k=min(1,Vlimit/|V*|)，Vlim=k*V*。
 * requested_v 保存限幅前电压，limited_v 保存限幅后电压，scale 输出 k。
 * 两轴乘同一个 k，所以保持矢量方向；不能把 d/q 分别截到 +/-Vlimit。
 * limit_v 是调用者按实际母线提供的圆形边界，零矢量无需缩放，k=1。
 */
int Foc_VoltageLimit(FocDq pi_v, FocDq feedforward_v, float limit_v,
                     FocDq *requested_v, FocDq *limited_v, float *scale)
{
    float peak;
    float norm;
    *limited_v = (FocDq){0.0f, 0.0f};
    *scale = 0.0f;
    requested_v->d = pi_v.d + feedforward_v.d;
    requested_v->q = pi_v.q + feedforward_v.q;
    if (!isfinite(requested_v->d) || !isfinite(requested_v->q) ||
        !isfinite(limit_v) || limit_v <= 0.0f) {
        return 0;
    }

    /* peak 是最大轴分量绝对值，不是矢量幅值；先归一化可避免大数直接平方溢出。 */
    peak = fmaxf(fabsf(requested_v->d), fabsf(requested_v->q));
    *scale = 1.0f;
    if (peak > 0.0f) {
        /* norm=|V*|/peak；下面的比较及缩放仍等价于讲义中的 |V*|>Vlimit 和 k。 */
        norm = sqrtf((requested_v->d / peak) * (requested_v->d / peak) +
                     (requested_v->q / peak) * (requested_v->q / peak));
        if (peak > limit_v / norm) {
            *scale = (limit_v / peak) / norm;
        }
    }
    limited_v->d = requested_v->d * *scale;
    limited_v->q = requested_v->q * *scale;
    return 1;
}

/*
 * 第 02 课“四、反 Park”：旋转 d/q 电压 V -> 静止 alpha/beta 电压 V。
 * [Valpha] = [cos(theta_e) -sin(theta_e)] [Vd]
 * [Vbeta ]   [sin(theta_e)  cos(theta_e)] [Vq]
 * 这是 Park 矩阵的逆（转置），正负号必须与 Foc_Park 配套；正交变换保持幅值。
 * SVPWM 的开关矢量固定在定子坐标中，所以要先转回 alpha/beta，不能将 d/q 直接配给 A/B 相。
 * 第 02/06 课的时序约定：这里 sin/cos 对应预计电压生效时刻，可能不同于采样角。
 */
FocAlphaBeta Foc_InversePark(FocDq voltage, float sin_theta, float cos_theta)
{
    FocAlphaBeta result;
    result.alpha = voltage.d * cos_theta - voltage.q * sin_theta;
    result.beta = voltage.d * sin_theta + voltage.q * cos_theta;
    return result;
}

/*
 * 第 02 课“五、两电平逆变器”及“六、SVPWM”：alpha/beta 电压 V -> 三相占空比。
 * 本算子属于独立算法：输入只读，结果写入调用者的 pwm，不访问业务全局或硬件。
 * 有效矢量按 A/B/C 上桥臂状态定义：V1=100、V2=110、V3=010、V4=011、V5=001、V6=101，
 * 分别位于 0/60/120/180/240/300°，幅值均为 2*Vdc/3；V0=000、V7=111 为零矢量。
 * 扇区 k 用 Vk 和 V(k+1) 合成（第六扇区的下一矢量为 V1）：
 * Vref*Tpwm = Vk*T1 + V(k+1)*T2，T0=Tpwm-T1-T2。
 * 下文 t1/t2/t0 都是除以 Tpwm 后的比例，不是秒；硬件驱动另行安排 PWM 时序和死区。
 * 成功返回 1；非法输入或超出作用时间范围返回 0，并保留中性输出。
 */
int Foc_Svpwm(FocAlphaBeta voltage, float bus_v, FocSvpwm *pwm)
{
    float alpha;
    float beta;
    float a;
    float b;
    float active_ratio;
    float half_zero;
    FocSvpwm result = {0, 0.0f, 0.0f, 1.0f, 0.5f, 0.5f, 0.5f};
    *pwm = result;
    if (!isfinite(voltage.alpha) || !isfinite(voltage.beta) ||
        !isfinite(bus_v) || bus_v <= 0.0f) {
        return 0;
    }

    /* ① 以母线电压归一化；a/b 是后续六个扇区解方程共同使用的投影项。 */
    alpha = voltage.alpha / bus_v;
    beta = voltage.beta / bus_v;
    a = 1.5f * alpha;
    b = FOC_SQRT3_OVER_2 * beta;
    if (!isfinite(a) || !isfinite(b)) {
        return 0;
    }
    if (alpha == 0.0f && beta == 0.0f) {
        return 1; /* 零矢量没有方向：sector=0，t0=1，三相占空比均为 0.5。 */
    }

    /* ② 判断 60° 扇区，不计算 atan2。beta=+/-sqrt(3)*alpha 的边界等价于 b=+/-a。
     * 理想角度区间为 [0,60)、[60,120)、...、[300,360)；alpha 负轴归第四扇区。
     * 浮点边界由下面的比较确定；边界处相邻扇区有一个有效时间为零，占空比连续。
     */
    if (beta == 0.0f) {
        result.sector = (alpha > 0.0f) ? 1 : 4;
    } else if (beta > 0.0f) {
        if (alpha >= 0.0f) {
            result.sector = (b < a) ? 1 : 2;
        } else {
            result.sector = (b > -a) ? 2 : 3;
        }
    } else {
        if (alpha < 0.0f) {
            result.sector = (b > a) ? 4 : 5;
        } else {
            result.sector = (b < -a) ? 5 : 6;
        }
    }

    /* ③ 解相邻有效矢量作用时间。
     * 第一扇区：alpha=(2/3)*t1+(1/3)*t2，beta=(sqrt(3)/3)*t2，
     * 因而 t1=(3/2)*alpha-(sqrt(3)/2)*beta=a-b，t2=sqrt(3)*beta=2*b。
     * 其余扇区按对应 Vk/V(k+1) 的 alpha/beta 分量解同一组二维线性方程。
     */
    switch (result.sector) {
    case 1: /* V1=100，V2=110。 */
        result.t1_ratio = a - b;
        result.t2_ratio = 2.0f * b;
        break;
    case 2: /* V2=110，V3=010。 */
        result.t1_ratio = a + b;
        result.t2_ratio = -a + b;
        break;
    case 3: /* V3=010，V4=011。 */
        result.t1_ratio = 2.0f * b;
        result.t2_ratio = -a - b;
        break;
    case 4: /* V4=011，V5=001。 */
        result.t1_ratio = -a + b;
        result.t2_ratio = -2.0f * b;
        break;
    case 5: /* V5=001，V6=101。 */
        result.t1_ratio = -a - b;
        result.t2_ratio = a - b;
        break;
    case 6: /* V6=101，V1=100。 */
        result.t1_ratio = -2.0f * b;
        result.t2_ratio = a + b;
        break;
    default:
        return 0;
    }

    /* ④ T0=Tpwm-T1-T2；仅容忍 2e-6 周期的浮点边界误差，不实现过调制。
     * 若 t1+t2 只因舍入略大于 1，则归一回边界；真正超出六边形范围时拒绝输出。
     */
    if (!isfinite(result.t1_ratio) || !isfinite(result.t2_ratio) ||
        result.t1_ratio < -0.000002f || result.t2_ratio < -0.000002f) {
        return 0;
    }
    result.t1_ratio = fmaxf(0.0f, result.t1_ratio);
    result.t2_ratio = fmaxf(0.0f, result.t2_ratio);
    active_ratio = result.t1_ratio + result.t2_ratio;
    if (!isfinite(active_ratio) || active_ratio > 1.000002f) {
        return 0;
    }
    if (active_ratio > 1.0f) {
        result.t1_ratio /= active_ratio;
        result.t2_ratio /= active_ratio;
        result.t0_ratio = 0.0f;
    } else {
        result.t0_ratio = 1.0f - active_ratio;
    }

    /* ⑤ V0=000 与 V7=111 各分配 T0/2。相 x 在有效矢量中为 1 时，
     * 把该矢量的作用比例加到 Dx；V7 使三相各增加 t0/2，V0 不增加导通时间。
     * 第一扇区因此得到 Da=t1+t2+t0/2、Db=t2+t0/2、Dc=t0/2。
     * 这给出对称七段调制所需的占空比；实际开关边沿仍由 PWM 驱动产生。
     */
    half_zero = 0.5f * result.t0_ratio;
    switch (result.sector) {
    case 1:
        result.duty_a = result.t1_ratio + result.t2_ratio + half_zero;
        result.duty_b = result.t2_ratio + half_zero;
        result.duty_c = half_zero;
        break;
    case 2:
        result.duty_a = result.t1_ratio + half_zero;
        result.duty_b = result.t1_ratio + result.t2_ratio + half_zero;
        result.duty_c = half_zero;
        break;
    case 3:
        result.duty_a = half_zero;
        result.duty_b = result.t1_ratio + result.t2_ratio + half_zero;
        result.duty_c = result.t2_ratio + half_zero;
        break;
    case 4:
        result.duty_a = half_zero;
        result.duty_b = result.t1_ratio + half_zero;
        result.duty_c = result.t1_ratio + result.t2_ratio + half_zero;
        break;
    case 5:
        result.duty_a = result.t2_ratio + half_zero;
        result.duty_b = half_zero;
        result.duty_c = result.t1_ratio + result.t2_ratio + half_zero;
        break;
    case 6:
        result.duty_a = result.t1_ratio + result.t2_ratio + half_zero;
        result.duty_b = half_zero;
        result.duty_c = result.t1_ratio + half_zero;
        break;
    default:
        return 0;
    }
    /* 时间比例已合法，这里只消除最终加法可能产生的浮点端点误差。 */
    result.duty_a = fminf(1.0f, fmaxf(0.0f, result.duty_a));
    result.duty_b = fminf(1.0f, fmaxf(0.0f, result.duty_b));
    result.duty_c = fminf(1.0f, fmaxf(0.0f, result.duty_c));
    *pwm = result;
    return 1;
}
