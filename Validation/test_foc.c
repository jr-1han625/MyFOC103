/* FOC 数值验收：编译实际工程源码，以解析解、坐标往返和线电压重构作判据。 */
#include "foc_control.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned checks;

/* 检查带源代码行号的断言，失败立即停止，不受 NDEBUG 影响。 */
static void check(int condition, int line)
{
    ++checks;
    if (!condition) {
        fprintf(stderr, "FAIL at test_foc.c:%d\n", line);
        exit(1);
    }
}
#define CHECK(condition) check((condition), __LINE__)

/* 浮点比较同时使用绝对和相对容差；非有限结果不视为通过。 */
static int close_value(float actual, double expected)
{
    return isfinite(actual) && fabs((double)actual - expected) <=
           0.00001 + fabs(expected) * 0.000003;
}

/* 配置仅用于数值验收，不能充当真实板卡标度或已整定的控制参数。 */
static void configure(void)
{
    Foc_Init();
    foc.calibration = (FocCurrentCalibration){2048.0f, 2048.0f, 0.01f, 0.01f, 4095};
    foc.gains_d = (FocPiGains){2.0f, 10.0f};
    foc.gains_q = (FocPiGains){2.0f, 10.0f};
    foc.period_s = 0.001f;
    foc.voltage_utilization = 0.9f;
    foc.reference_a = (FocDq){0.0f, 2.0f};
    foc.enabled = 1;
}

/* 验收零点、小数零点、两种极性、码值越界以及无效标度。 */
static void test_sampling(void)
{
    FocCurrentCalibration cal = {2048.0f, 2048.0f, 0.01f, 0.01f, 4095};
    FocPhaseCurrent current;
    CHECK(Foc_CurrentSample(2048, 2048, &cal, &current));
    CHECK(close_value(current.a, 0) && close_value(current.b, 0) && close_value(current.c, 0));
    CHECK(Foc_CurrentSample(2148, 1998, &cal, &current));
    CHECK(close_value(current.a, 1) && close_value(current.b, -0.5) && close_value(current.c, -0.5));
    cal.amp_per_count_b = -0.01f;
    cal.offset_a_count = 2048.5f;
    CHECK(Foc_CurrentSample(2148, 1998, &cal, &current));
    CHECK(close_value(current.a, 0.995) && close_value(current.b, 0.5));
    CHECK(!Foc_CurrentSample(4096, 1998, &cal, &current));
    CHECK(close_value(current.a, 0));
    cal.amp_per_count_b = NAN;
    CHECK(!Foc_CurrentSample(2048, 2048, &cal, &current));
    cal.amp_per_count_b = 0;
    CHECK(!Foc_CurrentSample(2048, 2048, &cal, &current));
    cal.amp_per_count_b = 1;
    cal.offset_a_count = -1;
    CHECK(!Foc_CurrentSample(2048, 2048, &cal, &current));
}

/* 一周 720 个角度的平衡三相正弦，理论 dq=(2,0)；另验纯 q 轴和坐标往返。 */
static void test_transforms(void)
{
    int k;
    const double pi = 3.14159265358979323846;
    for (k = 0; k < 720; ++k) {
        double theta = k * pi / 360.0;
        float s = (float)sin(theta);
        float c = (float)cos(theta);
        FocPhaseCurrent phase = {(float)(2 * cos(theta)),
            (float)(2 * cos(theta - 2 * pi / 3)), (float)(2 * cos(theta + 2 * pi / 3))};
        FocAlphaBeta ab = Foc_Clarke(phase);
        FocDq dq = Foc_Park(ab, s, c);
        FocAlphaBeta back = Foc_InversePark(dq, s, c);
        FocAlphaBeta q_ab = {-s, c};
        FocDq q_dq = Foc_Park(q_ab, s, c);
        CHECK(close_value(dq.d, 2) && close_value(dq.q, 0));
        CHECK(close_value(back.alpha, ab.alpha) && close_value(back.beta, ab.beta));
        CHECK(close_value(q_dq.d, 0) && close_value(q_dq.q, 1));
    }
}

/* Ki 使用秒，验证连续两个周期的积分递推，避免漏乘 Ts 或重复积分。 */
static void test_pi(void)
{
    FocPiState state = {0};
    FocPiGains gains = {2, 10};
    CHECK(close_value(Foc_PiStep(2, 0.001f, gains, &state), 4.02));
    CHECK(close_value(state.integral_v, 0.02));
    CHECK(close_value(Foc_PiStep(-1, 0.001f, gains, &state), -1.99));
    CHECK(close_value(state.integral_v, 0.01));
}

/* 3-4-5 向量验方向保持、前馈合成、零矢量、边界和超大有限输入。 */
static void test_limit(void)
{
    FocDq requested, limited;
    float scale;
    CHECK(Foc_VoltageLimit((FocDq){2, 3}, (FocDq){1, 1}, 2.5f, &requested, &limited, &scale));
    CHECK(close_value(requested.d, 3) && close_value(requested.q, 4));
    CHECK(close_value(limited.d, 1.5) && close_value(limited.q, 2) && close_value(scale, 0.5));
    CHECK(Foc_VoltageLimit((FocDq){3, 4}, (FocDq){0, 0}, 5, &requested, &limited, &scale));
    CHECK(close_value(scale, 1));
    CHECK(Foc_VoltageLimit((FocDq){0, 0}, (FocDq){0, 0}, 5, &requested, &limited, &scale));
    CHECK(close_value(scale, 1) && close_value(limited.q, 0));
    CHECK(Foc_VoltageLimit((FocDq){FLT_MAX, FLT_MAX}, (FocDq){0, 0}, 10,
                          &requested, &limited, &scale));
    CHECK(close_value(limited.d, 10 / sqrt(2)) && close_value(limited.q, 10 / sqrt(2)));
    CHECK(!Foc_VoltageLimit((FocDq){1, 1}, (FocDq){0, 0}, 0, &requested, &limited, &scale));
}

/* 从占空比重构线电压，与解析三相正弦电压差比较；覆盖六扇区边界及其两侧。 */
static void test_svpwm(void)
{
    int k, side;
    const double pi = 3.14159265358979323846;
    const float bus = 24;
    FocSvpwm pwm;
    CHECK(Foc_Svpwm((FocAlphaBeta){0, 0}, bus, &pwm));
    CHECK(pwm.sector == 0 && close_value(pwm.t1_ratio, 0) &&
          close_value(pwm.t2_ratio, 0) && close_value(pwm.t0_ratio, 1));
    CHECK(close_value(pwm.duty_a, 0.5) && close_value(pwm.duty_b, 0.5) && close_value(pwm.duty_c, 0.5));
    CHECK(Foc_Svpwm((FocAlphaBeta){6, 0}, bus, &pwm));
    CHECK(pwm.sector == 1 && close_value(pwm.t1_ratio, 0.375) &&
          close_value(pwm.t2_ratio, 0) && close_value(pwm.t0_ratio, 0.625));
    CHECK(close_value(pwm.duty_a, 0.6875) && close_value(pwm.duty_b, 0.3125) && close_value(pwm.duty_c, 0.3125));
    for (k = 0; k <= 720; ++k) {
        for (side = -1; side <= 1; ++side) {
            double theta = k * pi / 360.0 + side * 1e-6;
            double magnitude = bus / sqrt(3.0);
            FocAlphaBeta ab = {(float)(magnitude * cos(theta)), (float)(magnitude * sin(theta))};
            double va = magnitude * cos(theta);
            double vb = magnitude * cos(theta - 2 * pi / 3);
            double vc = magnitude * cos(theta + 2 * pi / 3);
            CHECK(Foc_Svpwm(ab, bus, &pwm));
            CHECK(pwm.sector >= 1 && pwm.sector <= 6);
            CHECK(pwm.t1_ratio >= 0 && pwm.t2_ratio >= 0 && pwm.t0_ratio >= 0);
            CHECK(close_value(pwm.t1_ratio + pwm.t2_ratio + pwm.t0_ratio, 1));
            CHECK(pwm.duty_a >= 0 && pwm.duty_a <= 1 && pwm.duty_b >= 0 &&
                  pwm.duty_b <= 1 && pwm.duty_c >= 0 && pwm.duty_c <= 1);
            CHECK(close_value(fmaxf(pwm.duty_a, fmaxf(pwm.duty_b, pwm.duty_c)) +
                              fminf(pwm.duty_a, fminf(pwm.duty_b, pwm.duty_c)), 1));
            CHECK(close_value((pwm.duty_a - pwm.duty_b) * bus, va - vb));
            CHECK(close_value((pwm.duty_b - pwm.duty_c) * bus, vb - vc));
            CHECK(close_value((pwm.duty_c - pwm.duty_a) * bus, vc - va));
        }
    }
    CHECK(!Foc_Svpwm((FocAlphaBeta){100, 0}, bus, &pwm));
    CHECK(pwm.sector == 0 && close_value(pwm.t0_ratio, 1) && close_value(pwm.duty_a, 0.5));
    CHECK(!Foc_Svpwm((FocAlphaBeta){0, 0}, 0, &pwm));
    CHECK(!Foc_Svpwm((FocAlphaBeta){NAN, 0}, bus, &pwm));
    CHECK(!Foc_Svpwm((FocAlphaBeta){FLT_MAX, FLT_MAX}, 0.001f, &pwm));
}

/* 六个扇区中心的独立正弦解析解，核对扇区号、T1/T2/T0 和高/中/低相分配。 */
static void test_sector_centers(void)
{
    const double pi = 3.14159265358979323846;
    const double t = sqrt(3.0) / 6.0; /* Vref=4 V，Vdc=12 V，局部角 30°。 */
    const double expected[6][3] = {
        {0.7886751345948129, 0.5, 0.2113248654051871},
        {0.5, 0.7886751345948129, 0.2113248654051871},
        {0.2113248654051871, 0.7886751345948129, 0.5},
        {0.2113248654051871, 0.5, 0.7886751345948129},
        {0.5, 0.2113248654051871, 0.7886751345948129},
        {0.7886751345948129, 0.2113248654051871, 0.5}
    };
    int k;
    FocSvpwm pwm;
    for (k = 0; k < 6; ++k) {
        double theta = (30 + 60 * k) * pi / 180;
        FocAlphaBeta ab = {(float)(4 * cos(theta)), (float)(4 * sin(theta))};
        CHECK(Foc_Svpwm(ab, 12, &pwm));
        CHECK(pwm.sector == k + 1);
        CHECK(close_value(pwm.t1_ratio, t) && close_value(pwm.t2_ratio, t));
        CHECK(close_value(pwm.t0_ratio, 1 - 2 * t));
        CHECK(close_value(pwm.duty_a, expected[k][0]));
        CHECK(close_value(pwm.duty_b, expected[k][1]));
        CHECK(close_value(pwm.duty_c, expected[k][2]));
    }
    CHECK(Foc_Svpwm((FocAlphaBeta){-4, 0}, 12, &pwm) && pwm.sector == 4);
    CHECK(Foc_Svpwm((FocAlphaBeta){0, 4}, 12, &pwm) && pwm.sector == 2);
    CHECK(Foc_Svpwm((FocAlphaBeta){0, -4}, 12, &pwm) && pwm.sector == 5);
}

/* 由实际开关状态的空间矢量重构输入，独立验证作用时间，而不复制六扇区计算公式。 */
static void test_vector_reconstruction(void)
{
    const double pi = 3.14159265358979323846;
    const double radius[] = {0.000001, 0.05, 0.5, 0.999, 1};
    const double vx[6] = {2.0/3, 1.0/3, -1.0/3, -2.0/3, -1.0/3, 1.0/3};
    const double vy[6] = {0, 0.5773502691896258, 0.5773502691896258,
                         0, -0.5773502691896258, -0.5773502691896258};
    int r, k;
    FocSvpwm pwm;
    for (r = 0; r < 5; ++r) {
        for (k = 0; k < 720; ++k) {
            double theta = (k + 0.25) * pi / 360; /* 避开边界，扇区号有唯一解析答案。 */
            double magnitude = radius[r] * 12 / sqrt(3.0);
            FocAlphaBeta ab = {(float)(magnitude * cos(theta)), (float)(magnitude * sin(theta))};
            int low, high;
            double alpha, beta;
            CHECK(Foc_Svpwm(ab, 12, &pwm));
            CHECK(pwm.sector == k / 120 + 1);
            low = pwm.sector - 1;
            high = (low + 1) % 6;
            alpha = 12 * (vx[low] * pwm.t1_ratio + vx[high] * pwm.t2_ratio);
            beta = 12 * (vy[low] * pwm.t1_ratio + vy[high] * pwm.t2_ratio);
            CHECK(close_value(ab.alpha, alpha));
            CHECK(close_value(ab.beta, beta));
            CHECK(close_value((pwm.duty_a - 0.5f * pwm.duty_b - 0.5f * pwm.duty_c) * 8, ab.alpha));
            CHECK(close_value((pwm.duty_b - pwm.duty_c) * 12 * FOC_INV_SQRT3, ab.beta));
        }
    }
    /* 六边形顶点属于可实现范围，单独算法不把整个六边形裁成上游使用的内切圆。 */
    for (k = 0; k < 6; ++k) {
        FocAlphaBeta vertex = {(float)(12 * vx[k]), (float)(12 * vy[k])};
        CHECK(Foc_Svpwm(vertex, 12, &pwm));
        CHECK(close_value(pwm.t0_ratio, 0));
        CHECK(!Foc_Svpwm((FocAlphaBeta){vertex.alpha * 1.001f, vertex.beta * 1.001f}, 12, &pwm));
    }
    CHECK(Foc_Svpwm((FocAlphaBeta){8.0f * 1.000001f, 0}, 12, &pwm));
    CHECK(close_value(pwm.t0_ratio, 0) && close_value(pwm.duty_a, 1));
    CHECK(!Foc_Svpwm((FocAlphaBeta){8.0f * 1.00001f, 0}, 12, &pwm));
}

/* 相邻扇区边界两侧应保持占空连续；只允许扇区号切换，不允许脉宽突变。 */
static void test_sector_boundaries(void)
{
    const double pi = 3.14159265358979323846;
    int k;
    for (k = 0; k < 6; ++k) {
        double angle = k * pi / 3;
        FocAlphaBeta before = {(float)(5 * cos(angle - 1e-5)), (float)(5 * sin(angle - 1e-5))};
        FocAlphaBeta after = {(float)(5 * cos(angle + 1e-5)), (float)(5 * sin(angle + 1e-5))};
        FocSvpwm left, right;
        CHECK(Foc_Svpwm(before, 12, &left) && Foc_Svpwm(after, 12, &right));
        CHECK(left.sector == (k == 0 ? 6 : k) && right.sector == k + 1);
        CHECK(fabsf(left.duty_a - right.duty_a) < 0.00002f);
        CHECK(fabsf(left.duty_b - right.duty_b) < 0.00002f);
        CHECK(fabsf(left.duty_c - right.duty_c) < 0.00002f);
    }
}

/* 全链可手算案例，故意让采样角与输出角不同，核实两条角度路径。 */
static void test_chain(void)
{
    FocSample sample = {2148, 1998, 12, 0, 1.5707963267948966f};
    configure();
    CHECK(Foc_ControlStep(&sample) == FOC_OK && foc.valid);
    CHECK(close_value(foc.current_a.a, 1) && close_value(foc.current_a.b, -0.5));
    CHECK(close_value(foc.current_ab_a.alpha, 1) && close_value(foc.current_ab_a.beta, 0));
    CHECK(close_value(foc.current_dq_a.d, 1) && close_value(foc.current_dq_a.q, 0));
    CHECK(close_value(foc.error_a.d, -1) && close_value(foc.error_a.q, 2));
    CHECK(close_value(foc.pi_v.d, -2.01) && close_value(foc.pi_v.q, 4.02));
    CHECK(close_value(foc.pi_d.integral_v, -0.01) && close_value(foc.pi_q.integral_v, 0.02));
    CHECK(close_value(foc.voltage_scale, 1));
    CHECK(close_value(foc.voltage_ab_v.alpha, -4.02) && close_value(foc.voltage_ab_v.beta, -2.01));
    CHECK(close_value(foc.pwm.duty_a, 0.17622037243));
    CHECK(close_value(foc.pwm.duty_b, 0.53366111730));
    CHECK(close_value(foc.pwm.duty_c, 0.82377962757));
    CHECK(foc.pwm.sector == 4);
    CHECK(close_value(foc.pwm.t1_ratio, 0.35744074487));
    CHECK(close_value(foc.pwm.t2_ratio, 0.29011851027));
    CHECK(close_value(foc.pwm.t0_ratio, 0.35244074487));
    printf("CHAIN: Ia=%.6f Ib=%.6f Id=%.6f Iq=%.6f Vd=%.6f Vq=%.6f Da=%.6f Db=%.6f Dc=%.6f\n",
           foc.current_a.a, foc.current_a.b, foc.current_dq_a.d, foc.current_dq_a.q,
           foc.limited_v.d, foc.limited_v.q, foc.pwm.duty_a, foc.pwm.duty_b, foc.pwm.duty_c);
}

/* 长期饱和不累积积分，并验证母线跌落、停机清零和重新使能。 */
static void test_saturation(void)
{
    int k;
    FocSample sample = {2048, 2048, 24, 0, 0};
    float previous_integral;
    configure();
    foc.reference_a = (FocDq){100, -100};
    foc.feedforward_v = (FocDq){2, -3};
    for (k = 0; k < 2000; ++k) {
        CHECK(Foc_ControlStep(&sample) == FOC_OK);
        CHECK(hypotf(foc.limited_v.d, foc.limited_v.q) <= foc.voltage_limit_v + 0.00001f);
        CHECK(close_value(foc.pi_d.integral_v, foc.limited_v.d - 200 - 2));
        CHECK(close_value(foc.pi_q.integral_v, foc.limited_v.q + 200 + 3));
    }
    previous_integral = foc.pi_d.integral_v;
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
    CHECK(close_value(foc.pi_d.integral_v, previous_integral));
    sample.bus_v = 6;
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
    CHECK(hypotf(foc.limited_v.d, foc.limited_v.q) <= 6 * 0.9 / sqrt(3.0) + 0.00001);
    foc.enabled = 0;
    CHECK(Foc_ControlStep(&sample) == FOC_DISABLED && !foc.valid);
    CHECK(close_value(foc.pi_d.integral_v, 0) && close_value(foc.pi_q.integral_v, 0));
    foc.enabled = 1;
    foc.reference_a = (FocDq){0, 0};
    foc.feedforward_v = (FocDq){0, 0};
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
    CHECK(close_value(foc.pwm.duty_a, 0.5));
}

/* 纯积分调节器达到上限后反转误差，应在下一周期退出上限并最终到达下限。 */
static void test_saturation_recovery(void)
{
    int k;
    FocSample sample = {2048, 2048, 24, 0, 0};
    float positive_limit;
    configure();
    foc.gains_d = (FocPiGains){0, 10};
    foc.reference_a = (FocDq){100, 0};
    for (k = 0; k < 100; ++k) {
        CHECK(Foc_ControlStep(&sample) == FOC_OK);
    }
    positive_limit = foc.voltage_limit_v;
    CHECK(close_value(foc.pi_d.integral_v, positive_limit));
    foc.reference_a.d = -100;
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
    CHECK(close_value(foc.pi_d.integral_v, positive_limit - 1));
    CHECK(close_value(foc.voltage_scale, 1));
    for (k = 0; k < 100; ++k) {
        CHECK(Foc_ControlStep(&sample) == FOC_OK);
    }
    CHECK(close_value(foc.pi_d.integral_v, -positive_limit));
}

/* 独立 RL 模型仅在测试中模拟零速电流动态，验证闭环收敛；不冒充真实电机。 */
static void test_rl_loop(void)
{
    int k;
    double iq = 0;
    const double decay = exp(-0.0001 / 0.002); /* R=1 ohm，L=2 mH，Ts=100 us。 */
    FocSample sample = {2048, 2048, 24, 0, 0};
    configure();
    foc.period_s = 0.0001f;
    foc.gains_d = (FocPiGains){2, 500};
    foc.gains_q = (FocPiGains){2, 500};
    for (k = 0; k < 10000; ++k) {
        /* theta=0、id=0 时，Ia=0，Ib=sqrt(3)/2*Iq；量化成真实 ADC 输入接口。 */
        sample.raw_b = (uint16_t)lround(2048 + sqrt(3.0) * iq / 0.02);
        CHECK(Foc_ControlStep(&sample) == FOC_OK);
        iq = iq * decay + foc.limited_v.q * (1 - decay);
    }
    CHECK(fabs(iq - 2) < 0.02);
    CHECK(fabs(foc.current_dq_a.d) < 0.00001);
    printf("RL MODEL: Iq=%.6f A (reference 2 A)\n", iq);
}

/* 失败必须清除旧有效标志、积分和占空结果，修复输入后允许重新计算。 */
static void test_rejection(void)
{
    FocSample sample = {2148, 1998, 24, 0, 0};
    configure();
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
    sample.bus_v = 0;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_SAMPLE && !foc.valid);
    CHECK(close_value(foc.pi_d.integral_v, 0) && close_value(foc.pwm.duty_a, 0.5));
    sample.bus_v = 24;
    sample.sample_angle_rad = NAN;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_SAMPLE);
    sample.sample_angle_rad = 0;
    sample.raw_a = 4096;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_SAMPLE);
    sample.raw_a = 2148;
    foc.period_s = 0;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_CONFIG);
    foc.period_s = 0.001f;
    foc.voltage_utilization = 1.1f;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_CONFIG);
    foc.voltage_utilization = 0.9f;
    foc.gains_d.kp = -1;
    CHECK(Foc_ControlStep(&sample) == FOC_INVALID_CONFIG);
    foc.gains_d.kp = FLT_MAX;
    foc.reference_a.d = FLT_MAX;
    CHECK(Foc_ControlStep(&sample) == FOC_NUMERIC_ERROR && !foc.valid);
    configure();
    CHECK(Foc_ControlStep(&sample) == FOC_OK);
}

/* 主机验收入口，调用的就是加入 Keil 工程的同一份 C 实现。 */
int main(void)
{
    test_sampling();
    test_transforms();
    test_pi();
    test_limit();
    test_svpwm();
    test_sector_centers();
    test_vector_reconstruction();
    test_sector_boundaries();
    test_chain();
    test_saturation();
    test_saturation_recovery();
    test_rl_loop();
    test_rejection();
    printf("PASS: %u checks (sampling/transforms/PI/limit/SVPWM/chain/saturation/RL/rejection)\n", checks);
    return 0;
}
