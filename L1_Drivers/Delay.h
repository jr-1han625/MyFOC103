/* L1 Delay：延时单位换算和分段执行；硬件提供微秒等待。
 * 先由 L4 调用 B.bind 绑定静态 A 表，再调用初始化及软件能力。
 * A 表及函数须在整个驱动使用期有效；绑定/初始化只在启动阶段执行，不得并发改绑。
 * 使用 SysTick 的实现会独占并改写计数器，不能与另一 SysTick 调度器同时使用。
 */
#ifndef DRIVER_DELAY_H
#define DRIVER_DELAY_H
#include <stdint.h>

/* A：由硬件实现的同步能力，参数不暴露寄存器或物理引脚。 */
typedef struct {
    void (*wait_us)(uint32_t us); /* 微秒硬件等待，原 72 MHz 时基，单次 0~233015 us。 */
} Delay_Hardware;

/* B：可移植软件能力；init 负责触发已绑定硬件初始化及本模块软件初始化。 */
typedef struct {
    void (*bind)(const Delay_Hardware *hardware); /* 只读 A 表，启动时绑定一次。 */
    void (*us)(uint32_t us); /* Delay_us 的软件能力，输入范围见下方约定。 */
    void (*ms)(uint32_t ms); /* Delay_ms 的软件能力，输入范围见下方约定。 */
    void (*seconds)(uint32_t s); /* Delay_s 的软件能力，输入范围见下方约定。 */
} Delay_Software;
extern const Delay_Software Delay_B;

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
