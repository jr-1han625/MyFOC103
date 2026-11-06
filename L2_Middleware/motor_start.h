/* L2 首次空载试转：零点平均、等待、定位、开环加速、限时运行及故障停机。
 * Motor_StartInit 在采样前调用；Motor_StartStep 每个 PWM 采样帧调用一次。
 * motor_start 为唯一公开状态，外部只读；不要在运行中用调试器写其字段。
 * 不测量转速，不检测失步，不投入电流 PI；7 对极仅用于指令速度换算。
 */
#ifndef MOTOR_START_H
#define MOTOR_START_H
#include "MotorIo.h"
#include "foc_math.h"
#include "foc_openloop.h"

typedef enum {
    MOTOR_ZERO = 0, MOTOR_WAIT, MOTOR_ALIGN, MOTOR_RUN, MOTOR_DONE
} MotorStage; /* ADC 入口拥有的启动阶段；任意故障优先于阶段显示。 */
typedef enum {
    MOTOR_FAULT_NONE = 0, MOTOR_FAULT_INIT, MOTOR_FAULT_DRIVER,
    MOTOR_FAULT_TIMING, MOTOR_FAULT_BUS, MOTOR_FAULT_ZERO,
    MOTOR_FAULT_CURRENT, MOTOR_FAULT_SAMPLE, MOTOR_FAULT_OUTPUT
} MotorFault; /* 首次故障锁存，复位重新上电后才清除。 */

/* 启动模块公开观测数据；ISR 更新，前台只读取单个字段显示。 */
typedef struct {
    volatile MotorStage stage; /* 当前阶段。 */
    volatile MotorFault fault; /* 首个故障原因，0 无故障。 */
    uint32_t stage_ticks; /* 当前阶段采样帧数，20 kHz，切阶段清零。 */
    float offset_a; /* A 相零电流平均 ADC 码。 */
    float offset_b; /* B 相零电流平均 ADC 码。 */
    volatile float bus_v; /* V，同步序列第二位的母线电压。 */
    volatile float bus_a; /* A，母线正向电流名义值。 */
    float sensor_a; /* A，A 相传感器有符号偏差，物理电流正向未确认。 */
    float sensor_b; /* A，B 相传感器有符号偏差。 */
    FocOpenLoopState angle; /* 角度算子历史；电频率和电周相位。 */
    float voltage_v; /* 当前电压矢量幅值 V。 */
    FocSvpwm pwm; /* 最近一次扇区、时间比例及占空比。 */
} MotorStart;
extern MotorStart motor_start;
void Motor_StartInit(void);
void Motor_StartStep(const MotorIoSample *sample);
/* 可被更高优先级 ISR 调用：锁存故障并立即关断；不复位业务阶段或再次启动。 */
void Motor_Trip(MotorFault fault);
#endif
