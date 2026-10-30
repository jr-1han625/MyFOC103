#include "stm32f10x.h"
#include "board.h"

/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

/*引脚初始化*/
static void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
static void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
static void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
static void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void BSP_OLED_Command(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void BSP_OLED_Data(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void BSP_OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			//端口初始化
	
	BSP_OLED_Command(0xAE);	//关闭显示
	
	BSP_OLED_Command(0xD5);	//设置显示时钟分频比/振荡器频率
	BSP_OLED_Command(0x80);
	
	BSP_OLED_Command(0xA8);	//设置多路复用率
	BSP_OLED_Command(0x3F);
	
	BSP_OLED_Command(0xD3);	//设置显示偏移
	BSP_OLED_Command(0x00);
	
	BSP_OLED_Command(0x40);	//设置显示开始行
	
	BSP_OLED_Command(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	BSP_OLED_Command(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	BSP_OLED_Command(0xDA);	//设置COM引脚硬件配置
	BSP_OLED_Command(0x12);
	
	BSP_OLED_Command(0x81);	//设置对比度控制
	BSP_OLED_Command(0xCF);

	BSP_OLED_Command(0xD9);	//设置预充电周期
	BSP_OLED_Command(0xF1);

	BSP_OLED_Command(0xDB);	//设置VCOMH取消选择级别
	BSP_OLED_Command(0x30);

	BSP_OLED_Command(0xA4);	//设置整个显示打开/关闭

	BSP_OLED_Command(0xA6);	//设置正常/倒转显示

	BSP_OLED_Command(0x8D);	//设置充电泵
	BSP_OLED_Command(0x14);

	BSP_OLED_Command(0xAF);	//开启显示
		
}
