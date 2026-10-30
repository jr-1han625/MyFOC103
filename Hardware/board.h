/* L0 板级能力：仅供 L4 静态装配引用。配置与寄存器访问留在对应实现中。
 * OLED 使用 PB8/PB9，LED 使用 PA1/PA2，按键使用 PB1/PB11；迁移时在 L0 修改。
 * SysTick 等待沿用原 72 MHz 假设且独占 SysTick，不提供新的周期调度。
 */
#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>
void BSP_OLED_Init(void);
void BSP_OLED_Command(uint8_t command);
void BSP_OLED_Data(uint8_t data);
void BSP_LED_Init(void);
void BSP_LED_Write(uint8_t channel, uint8_t level);
uint8_t BSP_LED_Read(uint8_t channel);
void BSP_Key_Init(void);
uint8_t BSP_Key_Read(uint8_t channel);
void BSP_DelayUs(uint32_t us);
#endif
