#include "LED.h"

static const LED_Hardware *hw;

/* 绑定由 L4 持有的静态 A 表；只保存引用，不复制硬件状态。 */
static void LED_Bind(const LED_Hardware *hardware)
{
    hw = hardware;
}

/* 按原默认状态初始化硬件，不另建 LED 电平镜像。 */
void LED_Init(void)
{
    hw->init();
}

/* 设置通道 1 为点亮，通过 A 能力写原电平。 */
void LED1_ON(void)
{
    hw->write_level(1, 0);
}

/* 设置通道 1 为熄灭，通过 A 能力写原电平。 */
void LED1_OFF(void)
{
    hw->write_level(1, 1);
}

/* 读取通道 1 的真实输出锁存电平后翻转，保持原读写行为。 */
void LED1_Turn(void)
{
    if (hw->read_level(1) == 0) {
        hw->write_level(1, 1);
    } else {
        hw->write_level(1, 0);
    }
}

/* 设置通道 2 为点亮，通过 A 能力写原电平。 */
void LED2_ON(void)
{
    hw->write_level(2, 0);
}

/* 设置通道 2 为熄灭，通过 A 能力写原电平。 */
void LED2_OFF(void)
{
    hw->write_level(2, 1);
}

/* 读取通道 2 的真实输出锁存电平后翻转，保持原读写行为。 */
void LED2_Turn(void)
{
    if (hw->read_level(2) == 0) {
        hw->write_level(2, 1);
    } else {
        hw->write_level(2, 0);
    }
}

const LED_Software LED_B = {
    LED_Bind,
    LED_Init,
    LED1_ON,
    LED1_OFF,
    LED1_Turn,
    LED2_ON,
    LED2_OFF,
    LED2_Turn,
};
