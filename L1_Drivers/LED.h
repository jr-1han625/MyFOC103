/* L1 LED：LED 开关与翻转机制；通道 1/2，不保存 GPIO 状态镜像。
 * 先由 L4 调用 B.bind 绑定静态 A 表，再调用初始化及软件能力。
 * A 表及函数须在整个驱动使用期有效；绑定/初始化只在启动阶段执行，不得并发改绑。
 */
#ifndef DRIVER_LED_H
#define DRIVER_LED_H
#include <stdint.h>

/* A：由硬件实现的同步能力，参数不暴露寄存器或物理引脚。 */
typedef struct {
    void (*init)(void); /* 配置两路 LED 引脚并设置原默认电平。 */
    void (*write_level)(uint8_t channel, uint8_t level); /* 通道 1/2 写电平 0/1，原板低电平点亮。 */
    uint8_t (*read_level)(uint8_t channel); /* 返回通道 1/2 的输出锁存电平。 */
} LED_Hardware;

/* B：可移植软件能力；init 负责触发已绑定硬件初始化及本模块软件初始化。 */
typedef struct {
    void (*bind)(const LED_Hardware *hardware); /* 只读 A 表，启动时绑定一次。 */
    void (*init)(void); /* LED_Init 的软件能力，输入范围见下方约定。 */
    void (*on1)(void); /* LED1_ON 的软件能力，输入范围见下方约定。 */
    void (*off1)(void); /* LED1_OFF 的软件能力，输入范围见下方约定。 */
    void (*toggle1)(void); /* LED1_Turn 的软件能力，输入范围见下方约定。 */
    void (*on2)(void); /* LED2_ON 的软件能力，输入范围见下方约定。 */
    void (*off2)(void); /* LED2_OFF 的软件能力，输入范围见下方约定。 */
    void (*toggle2)(void); /* LED2_Turn 的软件能力，输入范围见下方约定。 */
} LED_Software;
extern const LED_Software LED_B;

void LED_Init(void);
void LED1_ON(void);
void LED1_OFF(void);
void LED1_Turn(void);
void LED2_ON(void);
void LED2_OFF(void);
void LED2_Turn(void);

#endif
