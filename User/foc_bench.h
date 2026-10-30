/*
 * 调试器单步验收入口，归启动/调度装配使用，不是实际 ADC 驱动或实时控制调度。
 * 暂停 MCU 后填写 foc 的配置/命令和 foc_bench.sample，最后置 step_requested=1。
 * 继续运行后主循环执行一次 Foc_ControlStep；输出在 foc 中查看，不写 PWM。
 * 手动触发的墙钟间隔不代表控制周期，PI 仍按填写的 period_s 计算一个离散步。
 */
#ifndef FOC_BENCH_H
#define FOC_BENCH_H

#include "foc_control.h"

/* 调度入口拥有的验收命令，调试器仅在 MCU 暂停时填写输入。 */
typedef struct {
    FocSample sample; /* 输入：下一次单步使用的 ADC 原始码、母线电压和电角度。 */
    volatile uint8_t step_requested; /* 输入：置 1 请求计算一次，主循环消费后清零。 */
} FocBench;

extern FocBench foc_bench;

#endif
