#include "driver_assembly.h"
#include "board.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Delay.h"

/* OLED 的只读硬件绑定，生命周期覆盖整个程序。 */
static const OLED_Hardware oled_hardware = {BSP_OLED_Init, BSP_OLED_Command, BSP_OLED_Data};
/* LED 的只读硬件绑定，生命周期覆盖整个程序。 */
static const LED_Hardware led_hardware = {BSP_LED_Init, BSP_LED_Write, BSP_LED_Read};
/* Key 的只读硬件绑定，生命周期覆盖整个程序。 */
static const Key_Hardware key_hardware = {BSP_Key_Init, BSP_Key_Read};
/* Delay 的只读硬件绑定，生命周期覆盖整个程序。 */
static const Delay_Hardware delay_hardware = {BSP_DelayUs};

/* 启动时完成全部静态绑定，不注册 ID、不初始化未使用的 GPIO。 */
void Drivers_Bind(void)
{
    OLED_B.bind(&oled_hardware);
    LED_B.bind(&led_hardware);
    Key_B.bind(&key_hardware);
    Delay_B.bind(&delay_hardware);
}
