/* L3：当前功率板与电机的配置入口；参数选择不等于外设已经实现。
 * 板卡硬件标度归 Hardware/power_board.h；这里选择项目工作参数并推导换算系数。
 * PI、零点及极性须调试确定，不能从功率板峰值电流或文档示例推断。
 */
#ifndef XY_SELECT_H
#define XY_SELECT_H
#include "power_board.h"

#define FOC_MOTOR_MAX_V 12.0f /* 用户提供的电机电压上限，不使用功率板 40 V 上限替代。 */
#define FOC_PWM_HZ 20000U /* 项目选定 20 kHz，位于手册 10~40 kHz 范围；尚无实际 PWM 调度。 */
#define FOC_ADC_MAX_COUNT 4095U /* STM32F103 12 位原始码的满码。 */
#define FOC_ADC_REFERENCE_V 3.3f /* MCU ADC 名义参考，必须实测 VDDA；不等同于自动接入板载 VREF。 */
#define FOC_PHASE_CURRENT_POLARITY 0 /* 待确认：+1/-1；0 明确表示未标定，电流换算应被拒绝。 */

#if FOC_PWM_HZ < POWER_BOARD_PWM_MIN_HZ || FOC_PWM_HZ > POWER_BOARD_PWM_MAX_HZ
#error FOC_PWM_HZ_outside_power_board_range
#endif
#if FOC_PHASE_CURRENT_POLARITY != -1 && FOC_PHASE_CURRENT_POLARITY != 0 && FOC_PHASE_CURRENT_POLARITY != 1
#error Invalid_phase_current_polarity
#endif
#define FOC_CONTROL_PERIOD_S (1.0f / FOC_PWM_HZ) /* 规划每 PWM 周期计算一次；实际接入后核对。 */
#define FOC_PHASE_AMP_PER_COUNT_MAGNITUDE (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_PHASE_SHUNT_OHM * POWER_BOARD_PHASE_GAIN))
#define FOC_BUS_VOLT_PER_COUNT (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_BUS_VOLTAGE_RATIO))
#define FOC_BUS_AMP_PER_COUNT (FOC_ADC_REFERENCE_V / \
    (FOC_ADC_MAX_COUNT * POWER_BOARD_BUS_SHUNT_OHM * POWER_BOARD_BUS_CURRENT_GAIN))
#endif
