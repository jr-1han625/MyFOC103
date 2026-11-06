/* L1 电机端口：占空比量化、输入检查和零点平均；硬件通过 A 表提供。
 * L4 启动时 bind/init/wake，随后仅由 ADC 中断串行调用采样、平均及输出。
 * stop 可由更高优先级故障中断调用；停机锁存后只能复位重新初始化。
 * 所有硬件表须静态存活；不包含 MCU、引脚或电机启动策略。
 */
#ifndef MOTOR_IO_H
#define MOTOR_IO_H
#include <stdint.h>

/* 一次注入序列快照：A/B 同时采样，母线电压/电流在下一序列位同时采样。 */
typedef struct {
    uint16_t phase_a; /* ADC A 相原始码，满码由硬件能力给出。 */
    uint16_t phase_b; /* ADC B 相原始码。 */
    uint16_t bus_voltage; /* 母线电压原始码。 */
    uint16_t bus_current; /* 母线电流原始码。 */
} MotorIoSample;

/* 硬件初始化输出的量化能力；L1 仅保存并使用，不猜测具体 MCU 的窗口。 */
typedef struct {
    uint16_t period_count; /* PWM 满占空比对应计数，非零。 */
    uint16_t adc_max_count; /* ADC 原始码满量程，非零。 */
    float duty_min; /* [0,1]，硬件采样窗口允许的最小占空比。 */
    float duty_max; /* [duty_min,1]，允许的最大占空比。 */
} MotorIoLimits;

/* A：硬件能力；0 为失败，1 为成功，所有操作均有确定的结束条件。 */
typedef struct {
    int (*init)(uint32_t pwm_hz, MotorIoLimits *limits); /* 配置并保持关断，输出硬件能力。 */
    int (*wake)(void); /* 六路全低下唤醒并完成芯片 CAL；不使能 PWM。 */
    void (*sample_start)(void); /* 启动同步采样及其超时监控。 */
    int (*read)(MotorIoSample *sample); /* ISR 读取完整帧并清完成标志，检查采样间隔。 */
    int (*write)(uint16_t a, uint16_t b, uint16_t c); /* 同时预装载三相比较值。 */
    int (*enable)(void); /* 检查硬件故障并开启六路互补输出。 */
    void (*stop)(void); /* 立即关 PWM、DRV_EN、CAL，并锁存关断。 */
    int (*finish)(void); /* ISR 末尾检查耗时/溢出并刷新硬件超时监控。 */
} MotorIoHardware;

/* B：软件能力；平均状态为驱动私有，输出零点由业务接收保存。 */
typedef struct {
    void (*bind)(const MotorIoHardware *hardware); /* 启动时绑定静态 A 表。 */
    int (*init)(uint32_t pwm_hz); /* 初始化硬件并重置平均状态。 */
    int (*wake)(void); /* 启动前调用一次；失败保持关断。 */
    void (*sample_start)(void); /* 启动每 PWM 周期一次的采样。 */
    int (*read)(MotorIoSample *sample); /* 读取并按硬件满码校验数据。 */
    int (*zero_average)(const MotorIoSample *sample, uint16_t count,
                        float *offset_a, float *offset_b); /* 连续平均 count 帧；完成返回 1，未完成 0。 */
    int (*write)(float a, float b, float c); /* 符合硬件 duty_min/max 的占空比；非法值关断。 */
    int (*enable)(void); /* 仅在已有有效占空比且未故障时开输出。 */
    void (*stop)(void); /* 锁存停止，复位后才允许再启动。 */
    int (*finish)(void); /* ADC ISR 最后调用，失败立即锁存停机。 */
} MotorIoSoftware;
extern const MotorIoSoftware MotorIo_B;
#endif
