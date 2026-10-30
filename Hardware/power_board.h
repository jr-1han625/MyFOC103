/* POWER_DRV8323HR_V1.01 板卡物理参数，来源 POWER_D..V1.pdf 第 3、5、6、8、9 页。
 * 这里只声明硬件标度与接口约束，不执行初始化、保护或采样。
 * 出厂为硬件电阻配置的 6PWM 模式，不能通过 MCU 软件直接切换为 3PWM。
 */
#ifndef POWER_BOARD_H
#define POWER_BOARD_H
#define POWER_BOARD_BUS_MIN_V 12.0f
#define POWER_BOARD_BUS_MAX_V 40.0f
#define POWER_BOARD_PWM_INPUTS 6
#define POWER_BOARD_PWM_MIN_HZ 10000U
#define POWER_BOARD_PWM_MAX_HZ 40000U
#define POWER_BOARD_WAKEUP_US 1000U /* DRV_EN 拉高后至少等待，再检查 FAULT。 */
#define POWER_BOARD_FAULT_ACTIVE_LEVEL 0U
#define POWER_BOARD_PHASE_SHUNT_OHM 0.004f
#define POWER_BOARD_PHASE_GAIN 5.0f
#define POWER_BOARD_BUS_SHUNT_OHM 0.004f
#define POWER_BOARD_BUS_CURRENT_GAIN 20.0f
#define POWER_BOARD_DIVIDER_TOP_OHM 124000.0f
#define POWER_BOARD_DIVIDER_BOTTOM_OHM 4700.0f
/* 串联输出电阻 R5=1k 不计入理想静态分压比；负载、ADC 采样时间需另行验证。 */
#define POWER_BOARD_BUS_VOLTAGE_RATIO (POWER_BOARD_DIVIDER_BOTTOM_OHM / \
    (POWER_BOARD_DIVIDER_TOP_OHM + POWER_BOARD_DIVIDER_BOTTOM_OHM))
/* 上电与停机必须六路 PWM 全低、DRV_EN/CAL 低；0.5 占空比不是关断状态。
 * FAULT 汇总故障无法区分类型；唤醒完成后低电平时禁止输出 PWM。
 * P2 的 +5.0V 与 VREF_3.3V 是板载输出，不允许外部向其输入电源。
 */
#endif
