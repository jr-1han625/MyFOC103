/* L1 OLED：显示排版、字模输出和数字格式化；硬件负责屏配置与总线事务。
 * 先由 L4 调用 B.bind 绑定静态 A 表，再调用初始化及软件能力。
 * A 表及函数须在整个驱动使用期有效；绑定/初始化只在启动阶段执行，不得并发改绑。
 * 显示接口仅供前台串行调用；字符、行列和数字长度范围沿用原接口，不增加裁剪。
 */
#ifndef DRIVER_OLED_H
#define DRIVER_OLED_H
#include <stdint.h>

/* A：由硬件实现的同步能力，参数不暴露寄存器或物理引脚。 */
typedef struct {
    void (*init)(void); /* 初始化 GPIO、执行原屏控制器配置和上电等待。 */
    void (*write_command)(uint8_t command); /* 同步发送一个屏控制命令字节。 */
    void (*write_data)(uint8_t data); /* 同步发送一个屏数据字节。 */
} OLED_Hardware;

/* B：可移植软件能力；init 负责触发已绑定硬件初始化及本模块软件初始化。 */
typedef struct {
    void (*bind)(const OLED_Hardware *hardware); /* 只读 A 表，启动时绑定一次。 */
    void (*init)(void); /* OLED_Init 的软件能力，输入范围见下方约定。 */
    void (*clear)(void); /* OLED_Clear 的软件能力，输入范围见下方约定。 */
    void (*show_char)(uint8_t Line, uint8_t Column, char Char); /* OLED_ShowChar 的软件能力，输入范围见下方约定。 */
    void (*show_string)(uint8_t Line, uint8_t Column, char *String); /* OLED_ShowString 的软件能力，输入范围见下方约定。 */
    void (*show_num)(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length); /* OLED_ShowNum 的软件能力，输入范围见下方约定。 */
    void (*show_signed_num)(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length); /* OLED_ShowSignedNum 的软件能力，输入范围见下方约定。 */
    void (*show_hex_num)(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length); /* OLED_ShowHexNum 的软件能力，输入范围见下方约定。 */
    void (*show_bin_num)(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length); /* OLED_ShowBinNum 的软件能力，输入范围见下方约定。 */
} OLED_Software;
extern const OLED_Software OLED_B;

/* 行 1~4、列 1~16；字符为可见 ASCII；字符串以零终止且须在行内。
 * 十进制长度 1~10、十六进制 1~8、二进制 1~16；与原接口相同，不做参数检查。 */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
