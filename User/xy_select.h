/* L3：A2212-KV930 首次空载试转配置；12 V、暂定 7 对极。
 * 板卡硬件标度归 Hardware/power_board.h；这里选择项目工作参数并推导换算系数。
 * PI、零点及极性须调试确定，不能从功率板峰值电流或文档示例推断。
 */
#ifndef XY_SELECT_H
#define XY_SELECT_H
#include "power_board.h"

#define FOC_MOTOR_MAX_V 12.0f /* 用户提供的电机电压上限，不使用功率板 40 V 上限替代。 */
#define FOC_PWM_HZ 20000U /* 项目选定 20 kHz，与当前 TIM1/ADC 窗口配置一致。 */
#define FOC_ADC_MAX_COUNT 4095U /* STM32F103 12 位原始码的满码。 */
#define FOC_ADC_REFERENCE_V 3.3f /* MCU ADC 名义参考，必须实测 VDDA；不等同于自动接入板载 VREF。 */
#define FOC_PHASE_CURRENT_POLARITY 0 /* 待确认：+1/-1；0 明确表示未标定，电流换算应被拒绝。 */

#if FOC_PWM_HZ < POWER_BOARD_PWM_MIN_HZ || FOC_PWM_HZ > POWER_BOARD_PWM_MAX_HZ
#error FOC_PWM_HZ_outside_power_board_range
#endif
#if FOC_PHASE_CURRENT_POLARITY != -1 && FOC_PHASE_CURRENT_POLARITY != 0 && FOC_PHASE_CURRENT_POLARITY != 1
#error Invalid_phase_current_polarity
#endif
/* 首次试转参数，不将用户估计的 30 A 当作额定值；带桨/带载前需另行整定。 */
#define FOC_MOTOR_POLE_PAIRS 7U /* 用户暂定；只影响指令机械转速换算。 */
#define FOC_TEST_SPEED_RPM 300.0f /* 指令机械 rpm；开环不能测量或保证实际转速。 */
#define FOC_TEST_CURRENT_LIMIT_A 3.0f /* 任一采样超限即锁存；不是恒流控制。 */
#define FOC_ALIGN_V 0.20f /* 定位矢量幅值 V；从零缓升。 */
#define FOC_RUN_V 0.65f /* 300 rpm 时矢量幅值 V，需实机按电机调整。 */
#define FOC_ELECTRICAL_ACCEL_HZ_S 12.0f /* Hz/s，35 Hz 电频率约 2.92 s 到达。 */
#define FOC_BUS_TRIP_LOW_V 11.5f /* 采样关断阈值，留 ADC/纹波余量；供电仍要求标称 12 V。 */
#define FOC_BUS_TRIP_HIGH_V 12.5f /* 采样关断阈值，不是允许改用更高电压电源。 */
#define FOC_ZERO_SAMPLES 1024U
#define FOC_START_WAIT_TICKS (3U * FOC_PWM_HZ)
#define FOC_ALIGN_TICKS (FOC_PWM_HZ / 2U)
#define FOC_RUN_TICKS (10U * FOC_PWM_HZ) /* 含加速，共试转 10 s 后停机。 */
#define FOC_CONTROL_DIVIDER 4U
#define FOC_CONTROL_PERIOD_S ((float)FOC_CONTROL_DIVIDER / FOC_PWM_HZ)
#if FOC_PWM_HZ != 20000U
#error Motor_sampling_window_requires_20kHz
#endif
#define FOC_PHASE_AMP_PER_COUNT_MAGNITUDE (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_PHASE_SHUNT_OHM * POWER_BOARD_PHASE_GAIN))
#define FOC_BUS_VOLT_PER_COUNT (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_BUS_VOLTAGE_RATIO))
#define FOC_BUS_AMP_PER_COUNT (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_BUS_SHUNT_OHM * POWER_BOARD_BUS_CURRENT_GAIN))
#endif
