/* L4：初始化静态装配和硬件，启动同步采样；前台只显示状态。
 * 上电自动执行一次空载试转，完成或故障后需复位才再次启动。
 */
#include "driver_assembly.h"
#include "OLED.h"
#include "Delay.h"
#include "MotorIo.h"
#include "motor_start.h"
#include "foc_control.h"
#include "xy_select.h"

/* 中断负责采样和控制，显示/延时不放入实时入口。 */
int main(void)
{
    int ready;
    Drivers_Bind();
    Foc_Init(); /* 完整电流环保留供验收，当前没有真实转子角度，故保持禁用。 */
    Motor_StartInit();
    ready = MotorIo_B.init(FOC_PWM_HZ);
    OLED_B.init();
    OLED_B.show_string(1, 1, "Open Loop Test");
    OLED_B.show_string(2, 1, "Stage:  Fault:");
    OLED_B.show_string(3, 1, "Bus mV:");
    OLED_B.show_string(4, 1, "Reset to retry");
    if (!ready) {
        Motor_Trip(MOTOR_FAULT_INIT);
    } else if (!MotorIo_B.wake()) {
        Motor_Trip(MOTOR_FAULT_DRIVER);
    } else {
        MotorIo_B.sample_start();
    }
    while (1) {
        OLED_B.show_num(2, 7, (uint32_t)motor_start.stage, 1);
        OLED_B.show_num(2, 15, (uint32_t)motor_start.fault, 1);
        OLED_B.show_num(3, 9, (uint32_t)(motor_start.bus_v * 1000.0f), 5);
        Delay_B.ms(200);
    }
}
