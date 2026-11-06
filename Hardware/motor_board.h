/* L0：STM32F103C8T6 + POWER_DRV8323HR 的 PWM、双 ADC 和故障端口。
 * 供 L4 静态绑定到 MotorIo A 表；所有寄存器及物理引脚仅在实现中出现。
 * TIM1/ADC1/ADC2/TIM2 独占；TIM2 监控采样处理停止，DWT 测量 ISR 耗时。
 */
#ifndef MOTOR_BOARD_H
#define MOTOR_BOARD_H
#include "MotorIo.h"
int BSP_MotorInit(uint32_t pwm_hz, MotorIoLimits *limits);
int BSP_MotorWake(void);
void BSP_MotorSampleStart(void);
int BSP_MotorRead(MotorIoSample *sample);
int BSP_MotorWrite(uint16_t a, uint16_t b, uint16_t c);
int BSP_MotorEnable(void);
void BSP_MotorStop(void);
int BSP_MotorFinish(void);
/* L4 故障入口调用：先确认并清硬件标志，然后无条件关断。 */
int BSP_MotorBreak(void);
int BSP_MotorTimeout(void);
#endif
