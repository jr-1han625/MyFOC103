/* 主机验收：实际开环/启动/L1 源码接模拟硬件，不模拟真实电机是否跟随。 */
#include "motor_start.h"
#include "xy_select.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static unsigned checks, writes, stops;
static int hardware_stopped, enabled, wake_ok, write_ok, enable_ok, finish_ok;
static uint16_t compare_a, compare_b, compare_c;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static int Init(uint32_t hz, MotorIoLimits *limits)
{
    hardware_stopped = 0; enabled = 0;
    limits->period_count = 1800U;
    limits->adc_max_count = 4095U;
    limits->duty_min = 0.30f;
    limits->duty_max = 0.70f;
    return hz == 20000U;
}
static int Wake(void) { return wake_ok; }
static void Start(void) { }
static int Read(MotorIoSample *s) { (void)s; return !hardware_stopped; }
static int Write(uint16_t a, uint16_t b, uint16_t c)
{
    if (hardware_stopped || !write_ok) return 0;
    compare_a = a; compare_b = b; compare_c = c; ++writes; return 1;
}
static int Enable(void) { if (hardware_stopped || !enable_ok) return 0; enabled = 1; return 1; }
static void Stop(void) { hardware_stopped = 1; enabled = 0; ++stops; }
static int Finish(void) { return finish_ok; }
static const MotorIoHardware hardware = {Init, Wake, Start, Read, Write, Enable, Stop, Finish};

/* 每个用例重新上电，失败锁不能被普通调用重新开启。 */
static void Reset(void)
{
    writes = stops = 0;
    wake_ok = write_ok = enable_ok = finish_ok = 1;
    MotorIo_B.bind(&hardware);
    CHECK(MotorIo_B.init(FOC_PWM_HZ));
    CHECK(MotorIo_B.wake());
    Motor_StartInit();
}

/* 注入一帧真实业务输入，并检查功率输出的重构矢量和误差。 */
static void Frame(MotorIoSample sample)
{
    unsigned old_writes = writes;
    Motor_StartStep(&sample);
    if (motor_start.fault == MOTOR_FAULT_NONE && motor_start.stage != MOTOR_DONE) CHECK(MotorIo_B.finish());
    if (writes != old_writes) {
        double alpha = (2.0 * compare_a - compare_b - compare_c) * motor_start.bus_v / 5400.0;
        double beta = (compare_b - compare_c) * motor_start.bus_v / (1800.0 * sqrt(3.0));
        double theta = motor_start.angle.phase_turn * 6.283185307179586;
        CHECK(compare_a >= 540 && compare_a <= 1260);
        CHECK(compare_b >= 540 && compare_b <= 1260);
        CHECK(compare_c >= 540 && compare_c <= 1260);
        CHECK(fabs(alpha - motor_start.voltage_v * cos(theta)) < 0.008);
        CHECK(fabs(beta - motor_start.voltage_v * sin(theta)) < 0.008);
    }
}

static void TestAngle(void)
{
    unsigned i;
    FocOpenLoopState state = {0.0f, 0.0f}, original;
    FocOpenLoopOutput out;
    for (i = 0; i < 100000; ++i) {
        CHECK(Foc_OpenLoopStep(35.0f, 12.0f, 0.0002f, &state, &out));
        CHECK(state.phase_turn >= 0.0f && state.phase_turn < 1.0f);
        CHECK(fabs(out.sin_theta - sin(state.phase_turn * 6.283185307179586)) < 0.000077);
        CHECK(fabs(out.cos_theta - cos(state.phase_turn * 6.283185307179586)) < 0.000077);
    }
    CHECK(state.frequency_hz == 35.0f);
    for (i = 0; i < 20000; ++i) CHECK(Foc_OpenLoopStep(0.0f, 12.0f, 0.0002f, &state, &out));
    CHECK(state.frequency_hz == 0.0f);
    original = state;
    CHECK(!Foc_OpenLoopStep(NAN, 12.0f, 0.0002f, &state, &out));
    CHECK(!Foc_OpenLoopStep(35.0f, INFINITY, 0.0002f, &state, &out));
    CHECK(!Foc_OpenLoopStep(35.0f, 12.0f, 0.0f, &state, &out));
    CHECK(!Foc_OpenLoopStep(5000.0f, 12.0f, 0.0002f, &state, &out));
    CHECK(memcmp(&state, &original, sizeof(state)) == 0);
}

int main(void)
{
    unsigned i;
    MotorIoSample sample = {2048U, 2048U, 544U, 0U};
    TestAngle();
    Reset();
    for (i = 0; i < FOC_ZERO_SAMPLES; ++i) Frame(sample);
    CHECK(motor_start.stage == MOTOR_WAIT && !enabled && writes == 0);
    CHECK(motor_start.offset_a == 2048.0f && motor_start.offset_b == 2048.0f);
    for (i = 0; i < FOC_START_WAIT_TICKS - 1U; ++i) Frame(sample);
    CHECK(!enabled);
    Frame(sample);
    CHECK(motor_start.stage == MOTOR_ALIGN && enabled);
    for (i = 0; i < FOC_ALIGN_TICKS; ++i) Frame(sample);
    CHECK(motor_start.stage == MOTOR_RUN);
    for (i = 0; i < FOC_RUN_TICKS; ++i) Frame(sample);
    CHECK(motor_start.stage == MOTOR_DONE && !enabled && hardware_stopped);
    CHECK(motor_start.fault == MOTOR_FAULT_NONE);
    CHECK(motor_start.angle.frequency_hz == 35.0f);
    CHECK(!MotorIo_B.enable());
    CHECK(!MotorIo_B.write(0.5f, 0.5f, 0.5f));

    Reset(); sample.bus_voltage = 0; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_BUS && hardware_stopped);
    Reset(); sample.bus_voltage = 600; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_BUS);
    Reset(); sample.bus_voltage = 544; sample.phase_a = 0; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_SAMPLE);
    Reset(); sample.phase_a = 1400;
    for (i = 0; i < FOC_ZERO_SAMPLES; ++i) Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_ZERO);
    Reset(); sample.phase_a = 2048;
    for (i = 0; i < FOC_ZERO_SAMPLES; ++i) Frame(sample);
    sample.phase_a = 2123; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_CURRENT && hardware_stopped);
    Reset(); sample.phase_a = 2048;
    for (i = 0; i < FOC_ZERO_SAMPLES; ++i) Frame(sample);
    sample.phase_a = 2088; sample.phase_b = 2088; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_CURRENT); /* 重构 C 相超限。 */
    Reset(); sample.phase_a = sample.phase_b = 2048; sample.bus_current = 300; Frame(sample);
    CHECK(motor_start.fault == MOTOR_FAULT_CURRENT);
    Reset(); CHECK(!MotorIo_B.write(NAN, 0.5f, 0.5f)); CHECK(hardware_stopped && writes == 0);
    Reset(); CHECK(!MotorIo_B.write(0.5f, 0.71f, 0.5f)); CHECK(hardware_stopped);
    Reset(); wake_ok = 0; CHECK(!MotorIo_B.wake()); CHECK(hardware_stopped);
    Reset(); CHECK(!MotorIo_B.enable()); CHECK(hardware_stopped); /* 未先准备输出。 */
    Reset(); CHECK(MotorIo_B.write(0.5f, 0.5f, 0.5f));
    enable_ok = 0; CHECK(!MotorIo_B.enable()); CHECK(hardware_stopped);
    Reset(); write_ok = 0; CHECK(!MotorIo_B.write(0.5f, 0.5f, 0.5f)); CHECK(hardware_stopped);
    Reset(); finish_ok = 0; CHECK(!MotorIo_B.finish()); CHECK(hardware_stopped);
    Reset(); Motor_Trip(MOTOR_FAULT_DRIVER); Motor_Trip(MOTOR_FAULT_TIMING);
    CHECK(motor_start.fault == MOTOR_FAULT_DRIVER && hardware_stopped);
    CHECK(!MotorIo_B.write(0.5f, 0.5f, 0.5f));
    printf("PASS: %u checks; startup timing, vector reconstruction, cutoff, latch, angle lookup\n", checks);
    return 0;
}
