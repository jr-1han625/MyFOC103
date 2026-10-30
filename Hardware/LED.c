#include "stm32f10x.h"
#include "board.h"

/* 保留原时钟、引脚模式和初始电平。 */
void BSP_LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA1和PA2引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);				//设置PA1和PA2引脚为高电平
}

/* 通道 1/2 映射 PA1/PA2；非法通道不访问引脚。 */
void BSP_LED_Write(uint8_t channel, uint8_t level)
{
    uint16_t pin;
    if (channel != 1 && channel != 2) { return; }
    pin = (channel == 1) ? GPIO_Pin_1 : GPIO_Pin_2;
    GPIO_WriteBit(GPIOA, pin, level ? Bit_SET : Bit_RESET);
}
/* 读取硬件输出锁存器，非法通道返回高电平。 */
uint8_t BSP_LED_Read(uint8_t channel)
{
    if (channel != 1 && channel != 2) { return 1; }
    return GPIO_ReadOutputDataBit(GPIOA, channel == 1 ? GPIO_Pin_1 : GPIO_Pin_2);
}
