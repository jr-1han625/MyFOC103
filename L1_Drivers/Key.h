/* L1 Key：阻塞按键消抖与释放等待，保留原两键顺序及键 2 覆盖语义。
 * 先由 L4 调用 B.bind 绑定静态 A 表，再调用初始化及软件能力。
 * A 表及函数须在整个驱动使用期有效；绑定/初始化只在启动阶段执行，不得并发改绑。
 * 阻塞接口仅供前台使用，依赖已绑定的 Delay 软件驱动；原消抖每次 20 ms。
 */
#ifndef DRIVER_KEY_H
#define DRIVER_KEY_H
#include <stdint.h>

/* A：由硬件实现的同步能力，参数不暴露寄存器或物理引脚。 */
typedef struct {
    void (*init)(void); /* 配置两路按键输入及上拉。 */
    uint8_t (*read_level)(uint8_t channel); /* 读取通道 1/2 电平，0 表示按下。 */
} Key_Hardware;

/* B：可移植软件能力；init 负责触发已绑定硬件初始化及本模块软件初始化。 */
typedef struct {
    void (*bind)(const Key_Hardware *hardware); /* 只读 A 表，启动时绑定一次。 */
    void (*init)(void); /* Key_Init 的软件能力，输入范围见下方约定。 */
    uint8_t (*get_num)(void); /* Key_GetNum 的软件能力，输入范围见下方约定。 */
} Key_Software;
extern const Key_Software Key_B;

/* 返回 0 无按键，1/2 为键码；按住不放会阻塞，直到释放。 */
void Key_Init(void);
uint8_t Key_GetNum(void);

#endif
