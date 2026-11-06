#include "MotorIo.h"

static const MotorIoHardware *hw;
/* 私有量化与平均状态，只有本驱动更新；业务不访问其地址。 */
static struct {
    MotorIoLimits limits; /* 初始化取得的硬件量化和采样窗口能力。 */
    uint16_t zero_count; /* 本轮累计帧数，平均完成/初始化时清零。 */
    uint32_t zero_sum_a; /* 本轮 A 相原始码和。 */
    uint32_t zero_sum_b; /* 本轮 B 相原始码和。 */
    uint8_t written; /* 已成功准备过三相占空比，初始化清零。 */
    volatile uint8_t stopped; /* 故障/停机锁，高优先级中断也可置 1。 */
} io;

/* 保存静态硬件能力，必须早于其他接口。 */
static void Bind(const MotorIoHardware *hardware) { hw = hardware; }

/* 清零软件状态，硬件初始化成功后才允许输出准备。 */
static int Init(uint32_t pwm_hz)
{
    int ready = hw->init(pwm_hz, &io.limits);
    io.zero_count = 0;
    io.zero_sum_a = 0;
    io.zero_sum_b = 0;
    io.written = 0;
    io.stopped = !ready || io.limits.period_count == 0U || io.limits.adc_max_count == 0U ||
        !(io.limits.duty_min >= 0.0f && io.limits.duty_min <= io.limits.duty_max && io.limits.duty_max <= 1.0f);
    if (io.stopped) hw->stop();
    return !io.stopped;
}

/* 关断同时锁存，防止被打断的低优先级控制流程重新使能。 */
static void Stop(void) { io.stopped = 1; hw->stop(); }

/* 失败时统一撤销软件输出许可。 */
static int Wake(void)
{
    if (io.stopped || !hw->wake()) { Stop(); return 0; }
    return 1;
}

/* 采样时基由硬件产生，只在初始化完成后启动一次。 */
static void SampleStart(void) { if (!io.stopped) hw->sample_start(); }

/* 读取完整帧，原始码越界或硬件故障都锁存关断。 */
static int Read(MotorIoSample *sample)
{
    if (io.stopped || !hw->read(sample) || sample->phase_a > io.limits.adc_max_count ||
        sample->phase_b > io.limits.adc_max_count || sample->bus_voltage > io.limits.adc_max_count ||
        sample->bus_current > io.limits.adc_max_count) {
        Stop(); return 0;
    }
    return 1;
}

/* 累加零电流帧，到指定次数输出平均原始码并清累计值；count 必须非零。 */
static int ZeroAverage(const MotorIoSample *sample, uint16_t count, float *a, float *b)
{
    if (count == 0U) return 0;
    io.zero_sum_a += sample->phase_a;
    io.zero_sum_b += sample->phase_b;
    io.zero_count++;
    if (io.zero_count < count) return 0;
    *a = (float)io.zero_sum_a / io.zero_count;
    *b = (float)io.zero_sum_b / io.zero_count;
    io.zero_count = 0;
    io.zero_sum_a = 0;
    io.zero_sum_b = 0;
    return 1;
}

/* 先完整检查三相，再量化成同周期计数；比较形式同时拒绝 NaN 和无穷。 */
static int Write(float a, float b, float c)
{
    if (io.stopped || !(a >= io.limits.duty_min && a <= io.limits.duty_max) ||
        !(b >= io.limits.duty_min && b <= io.limits.duty_max) ||
        !(c >= io.limits.duty_min && c <= io.limits.duty_max)) {
        Stop(); return 0;
    }
    if (!hw->write((uint16_t)(a * io.limits.period_count + 0.5f),
                   (uint16_t)(b * io.limits.period_count + 0.5f),
                   (uint16_t)(c * io.limits.period_count + 0.5f))) {
        Stop(); return 0;
    }
    io.written = 1;
    return 1;
}

/* 输出使能必须晚于占空比准备，硬件仍独立校验故障锁。 */
static int Enable(void)
{
    if (io.stopped || !io.written || !hw->enable()) { Stop(); return 0; }
    return 1;
}

/* 处理完本帧后检查截止时间；停止后不再刷新超时监控。 */
static int Finish(void)
{
    if (io.stopped || !hw->finish()) { Stop(); return 0; }
    return 1;
}

const MotorIoSoftware MotorIo_B = {Bind, Init, Wake, SampleStart, Read,
                                  ZeroAverage, Write, Enable, Stop, Finish};
