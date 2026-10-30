/* L1 驱动模拟硬件验收：检查外部可见调用顺序和数据，无 STM32 头或寄存器依赖。 */
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, count, init_count, waits, delay_calls, delay_total;
static unsigned char trace[8192], reference[8192], levels[2];
static unsigned reads[2], held[2];
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d\n",__LINE__); exit(1); } } while (0)
/* 模拟初始化及每条命令/数据事务，保留命令类型与字节顺序。 */
static void init(void) { ++init_count; }
static void command(uint8_t value) { CHECK(count+2<=sizeof(trace)); trace[count++]=0; trace[count++]=value; }
static void data(uint8_t value) { CHECK(count+2<=sizeof(trace)); trace[count++]=1; trace[count++]=value; }
/* 模拟实际 LED 输出锁存电平。 */
static void write_level(uint8_t channel, uint8_t level) { CHECK(channel>=1 && channel<=2); levels[channel-1]=level; }
static uint8_t read_led(uint8_t channel) { CHECK(channel>=1 && channel<=2); return levels[channel-1]; }
/* 模拟按键先按下若干次读取后释放，用于检验阻塞等待和顺序。 */
static uint8_t read_key(uint8_t channel) { unsigned i=channel-1; ++reads[i]; return reads[i]<=held[i] ? 0 : 1; }
static void wait_us(uint32_t us) { ++waits; ++delay_calls; delay_total+=us; }
/* 比较数字格式化输出与已知字符序列的命令流，避免复写数字转换算法。 */
static void compare_text(const char *text)
{
    unsigned size=count;
    memcpy(reference,trace,size); count=0;
    OLED_B.show_string(1,1,(char *)text);
    CHECK(count==size && memcmp(reference,trace,size)==0);
    count=0;
}
/* 绑定静态模拟 A 表，执行与主工程相同的 L1 实现。 */
int main(void)
{
    static const OLED_Hardware oled={init,command,data};
    static const LED_Hardware led={init,write_level,read_led};
    static const Key_Hardware key={init,read_key};
    static const Delay_Hardware delay={wait_us};
    unsigned i;
    OLED_B.bind(&oled); LED_B.bind(&led); Key_B.bind(&key); Delay_B.bind(&delay);
    OLED_B.init();
    CHECK(init_count==1 && count==2*(8*3+1024));
    CHECK(trace[0]==0 && trace[1]==0xB0 && trace[3]==0x10 && trace[5]==0);
    for(i=0;i<8;++i) {
        unsigned base=i*(3+128)*2;
        CHECK(trace[base+1]==0xB0+i);
        CHECK(trace[base+6]==1 && trace[base+7]==0);
    }
    count=0;
    OLED_B.show_num(1,1,12345,5); compare_text("12345");
    OLED_B.show_signed_num(1,1,-66,2); compare_text("-66");
    OLED_B.show_hex_num(1,1,0xAA55,4); compare_text("AA55");
    OLED_B.show_bin_num(1,1,5,4); compare_text("0101");
    LED_B.init(); CHECK(init_count==2);
    LED_B.on1(); CHECK(levels[0]==0);
    LED_B.toggle1(); CHECK(levels[0]==1);
    LED_B.toggle1(); CHECK(levels[0]==0);
    LED_B.off1(); CHECK(levels[0]==1);
    LED_B.on2(); CHECK(levels[1]==0);
    LED_B.toggle2(); CHECK(levels[1]==1);
    LED_B.off2(); CHECK(levels[1]==1);
    Key_B.init(); CHECK(init_count==3);
    CHECK(Key_B.get_num()==0 && waits==0);
    memset(reads,0,sizeof(reads)); held[0]=3; held[1]=0;
    CHECK(Key_B.get_num()==1 && reads[0]==4 && waits==40);
    memset(reads,0,sizeof(reads)); held[0]=2; held[1]=2; waits=0;
    CHECK(Key_B.get_num()==2 && reads[0]==3 && reads[1]==3 && waits==80);
    delay_calls=delay_total=0;
    Delay_B.us(7); Delay_B.ms(2); Delay_B.seconds(1);
    CHECK(delay_calls==1003 && delay_total==1002007);
    Delay_B.ms(0); Delay_B.seconds(0); CHECK(delay_calls==1003);
    printf("PASS: %u checks; OLED stream, key debounce/priority, LED latch, delay units\n",checks);
    return 0;
}
