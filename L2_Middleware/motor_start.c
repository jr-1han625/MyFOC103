#include "motor_start.h"
#include "xy_select.h"
#include <string.h>
#include <math.h>

MotorStart motor_start;

/* 只在中断启动前初始化，清零算子历史，不开启功率输出。 */
void Motor_StartInit(void)
{
    memset(&motor_start, 0, sizeof(motor_start));
    motor_start.stage = MOTOR_ZERO;
}

/* 故障锁和物理关断共同防止中断抢占后的控制流程重新输出。 */
void Motor_Trip(MotorFault fault)
{
    if (motor_start.fault == MOTOR_FAULT_NONE) motor_start.fault = fault;
    MotorIo_B.stop();
}

/* 读取样本 -> 每帧保护 -> 启动阶段 -> 每 4 帧生成一次电压矢量。
 * 5 kHz 开环积分用实际 200 us 周期，20 kHz PWM 重复使用完整三相预装载值。
 */
void Motor_StartStep(const MotorIoSample *sample)
{
    float c;
    float target_hz;
    FocOpenLoopOutput trig;
    FocDq requested, limited, zero = {0.0f, 0.0f};
    FocAlphaBeta voltage;
    float scale;

    if (motor_start.fault != MOTOR_FAULT_NONE || motor_start.stage == MOTOR_DONE) return;
    motor_start.bus_v = sample->bus_voltage * FOC_BUS_VOLT_PER_COUNT;
    motor_start.bus_a = sample->bus_current * FOC_BUS_AMP_PER_COUNT;
    if (motor_start.bus_v < FOC_BUS_TRIP_LOW_V || motor_start.bus_v > FOC_BUS_TRIP_HIGH_V) {
        Motor_Trip(MOTOR_FAULT_BUS); return;
    }
    if (sample->phase_a < 100U || sample->phase_a > 3995U ||
        sample->phase_b < 100U || sample->phase_b > 3995U) {
        Motor_Trip(MOTOR_FAULT_SAMPLE); return;
    }
    if (motor_start.bus_a > FOC_TEST_CURRENT_LIMIT_A) {
        Motor_Trip(MOTOR_FAULT_CURRENT); return;
    }
    if (motor_start.stage == MOTOR_ZERO) {
        /* CAL 已拉低、增益恢复且六路 PWM 全关，平均 1024 帧真实零电流。 */
        if (MotorIo_B.zero_average(sample, FOC_ZERO_SAMPLES, &motor_start.offset_a, &motor_start.offset_b)) {
            if (motor_start.offset_a < 1600.0f || motor_start.offset_a > 2500.0f ||
                motor_start.offset_b < 1600.0f || motor_start.offset_b > 2500.0f) {
                Motor_Trip(MOTOR_FAULT_ZERO); return;
            }
            motor_start.stage = MOTOR_WAIT;
            motor_start.stage_ticks = 0;
        }
        return;
    }

    /* 极性未确认时仅取偏差绝对值；C 重构要求板上 A/B 两路同向增益。 */
    motor_start.sensor_a = (sample->phase_a - motor_start.offset_a) * FOC_PHASE_AMP_PER_COUNT_MAGNITUDE;
    motor_start.sensor_b = (sample->phase_b - motor_start.offset_b) * FOC_PHASE_AMP_PER_COUNT_MAGNITUDE;
    c = -motor_start.sensor_a - motor_start.sensor_b;
    if (fabsf(motor_start.sensor_a) > FOC_TEST_CURRENT_LIMIT_A ||
        fabsf(motor_start.sensor_b) > FOC_TEST_CURRENT_LIMIT_A || fabsf(c) > FOC_TEST_CURRENT_LIMIT_A) {
        Motor_Trip(MOTOR_FAULT_CURRENT); return;
    }
    motor_start.stage_ticks++;
    if (motor_start.stage == MOTOR_WAIT) {
        if (motor_start.stage_ticks < FOC_START_WAIT_TICKS) return;
        motor_start.stage = MOTOR_ALIGN;
        motor_start.stage_ticks = 0;
        motor_start.voltage_v = 0.0f;
        if (!MotorIo_B.write(0.5f, 0.5f, 0.5f) || !MotorIo_B.enable()) Motor_Trip(MOTOR_FAULT_OUTPUT);
        return;
    }
    if (motor_start.stage == MOTOR_ALIGN && motor_start.stage_ticks >= FOC_ALIGN_TICKS) {
        motor_start.stage = MOTOR_RUN;
        motor_start.stage_ticks = 0;
    }
    if (motor_start.stage == MOTOR_RUN && motor_start.stage_ticks >= FOC_RUN_TICKS) {
        motor_start.stage = MOTOR_DONE;
        MotorIo_B.stop();
        return;
    }
    if (motor_start.stage_ticks % FOC_CONTROL_DIVIDER != 0U) return;

    /* 定位时固定 alpha 方向；开始旋转后保持电压矢量连续，转子跟随旋转场。 */
    target_hz = (motor_start.stage == MOTOR_RUN) ? FOC_TEST_SPEED_RPM * FOC_MOTOR_POLE_PAIRS / 60.0f : 0.0f;
    if (!Foc_OpenLoopStep(target_hz, FOC_ELECTRICAL_ACCEL_HZ_S, FOC_CONTROL_PERIOD_S,
                         &motor_start.angle, &trig)) {
        Motor_Trip(MOTOR_FAULT_OUTPUT); return;
    }
    if (motor_start.stage == MOTOR_ALIGN) {
        motor_start.voltage_v += FOC_ALIGN_V / (FOC_ALIGN_TICKS / FOC_CONTROL_DIVIDER);
        if (motor_start.voltage_v > FOC_ALIGN_V) motor_start.voltage_v = FOC_ALIGN_V;
    } else {
        /* 随电频率由定位电压线性增加到试转电压，防止起步突然施加全幅值。 */
        motor_start.voltage_v = FOC_ALIGN_V + (FOC_RUN_V - FOC_ALIGN_V) * motor_start.angle.frequency_hz / target_hz;
    }
    requested.d = motor_start.voltage_v;
    requested.q = 0.0f;
    if (!Foc_VoltageLimit(requested, zero, motor_start.bus_v * FOC_INV_SQRT3 * 0.35f,
                          &requested, &limited, &scale)) {
        Motor_Trip(MOTOR_FAULT_OUTPUT); return;
    }
    voltage = Foc_InversePark(limited, trig.sin_theta, trig.cos_theta);
    if (!Foc_Svpwm(voltage, motor_start.bus_v, &motor_start.pwm) ||
        !MotorIo_B.write(motor_start.pwm.duty_a, motor_start.pwm.duty_b, motor_start.pwm.duty_c)) {
        Motor_Trip(MOTOR_FAULT_OUTPUT);
    }
}
