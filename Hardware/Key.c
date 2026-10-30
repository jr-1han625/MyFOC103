#include "stm32f10x.h"
#include "board.h"

/* 保留原时钟、引脚模式和初始电平。 */
void BSP_Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB1和PB11引脚初始化为上拉输入
}

/* 通道 1/2 映射 PB1/PB11；返回原始电平，非法通道视为释放。 */
uint8_t BSP_Key_Read(uint8_t channel)
{
    if (channel != 1 && channel != 2) { return 1; }
    return GPIO_ReadInputDataBit(GPIOB, channel == 1 ? GPIO_Pin_1 : GPIO_Pin_11);
}
