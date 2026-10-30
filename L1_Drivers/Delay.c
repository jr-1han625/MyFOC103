#include "Delay.h"

static const Delay_Hardware *hw;

/* 绑定由 L4 持有的静态 A 表；只保存引用，不复制硬件状态。 */
static void Delay_Bind(const Delay_Hardware *hardware)
{
    hw = hardware;
}

/* 微秒等待交给已绑定硬件；调用范围沿用原硬件限制。 */
void Delay_us(uint32_t xus)
{
    hw->wait_us(xus);
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

const Delay_Software Delay_B = {
    Delay_Bind,
    Delay_us,
    Delay_ms,
    Delay_s,
};
