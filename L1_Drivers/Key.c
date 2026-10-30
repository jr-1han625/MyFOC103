#include "Key.h"
#include "Delay.h"

static const Key_Hardware *hw;

/* 绑定由 L4 持有的静态 A 表；只保存引用，不复制硬件状态。 */
static void Key_Bind(const Key_Hardware *hardware)
{
    hw = hardware;
}

/* 触发已绑定的按键硬件初始化；消抖机制不持有硬件寄存器。 */
void Key_Init(void)
{
    hw->init();
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;		//定义变量，默认键码值为0
	
	if (hw->read_level(1) == 0)			//读按键通道1的状态，如果为0，则代表按键1按下
	{
		Delay_ms(20);											//延时消抖
		while (hw->read_level(1) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 1;												//置键码为1
	}
	
	if (hw->read_level(2) == 0)			//读按键通道2的状态，如果为0，则代表按键2按下
	{
		Delay_ms(20);											//延时消抖
		while (hw->read_level(2) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 2;												//置键码为2
	}
	
	return KeyNum;			//返回键码值，如果没有按键按下，所有if都不成立，则键码为默认值0
}

const Key_Software Key_B = {
    Key_Bind,
    Key_Init,
    Key_GetNum,
};
