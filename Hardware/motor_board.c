#include "motor_board.h"
#include "power_board.h"
#include "stm32f10x.h"

/* Cortex-M3 DWT 周期计数器；当前 CMSIS 版本没有 DWT 类型声明。 */
#define CPU_CYCLE_CTRL (*(volatile uint32_t *)0xE0001000UL)
#define CPU_CYCLE_COUNT (*(volatile uint32_t *)0xE0001004UL)
#define MOTOR_EN GPIO_Pin_10
#define MOTOR_CAL GPIO_Pin_5
#define MOTOR_FAULT GPIO_Pin_12

/* A/B/C 物理映射；TIM1 无重映射。上下桥均高有效，互补与死区由 TIM1 生成。 */
typedef struct {
    uint16_t high_pin; /* GPIOA 上桥引脚。 */
    uint16_t low_pin; /* GPIOB 下桥引脚。 */
    volatile uint16_t *compare; /* 对应 TIM1 预装载寄存器。 */
} MotorPhasePin;
static const MotorPhasePin phase_pins[3] = {
    {GPIO_Pin_8, GPIO_Pin_13, &TIM1->CCR1},
    {GPIO_Pin_9, GPIO_Pin_14, &TIM1->CCR2},
    {GPIO_Pin_10, GPIO_Pin_15, &TIM1->CCR3}
};
static volatile uint8_t stopped = 1;
static uint32_t frame_cycles;
static uint32_t irq_start_cycles;
static uint32_t last_sample_cycles;
static uint8_t have_sample;

/* DWT 计数下的有界微秒等待，仅用于器件唤醒和 ADC 启动。 */
static void WaitUs(uint32_t us)
{
    uint32_t start = CPU_CYCLE_COUNT;
    while ((uint32_t)(CPU_CYCLE_COUNT - start) < us * 72U) { }
}

/* ADC 内部校准等待上限 1 ms；失败向上返回，禁止卡在上电等待。 */
static int CalibrateAdc(ADC_TypeDef *adc)
{
    uint32_t start;
    ADC_Cmd(adc, ENABLE);
    WaitUs(10);
    ADC_ResetCalibration(adc);
    start = CPU_CYCLE_COUNT;
    while (ADC_GetResetCalibrationStatus(adc)) {
        if ((uint32_t)(CPU_CYCLE_COUNT - start) > 72000U) return 0;
    }
    ADC_StartCalibration(adc);
    start = CPU_CYCLE_COUNT;
    while (ADC_GetCalibrationStatus(adc)) {
        if ((uint32_t)(CPU_CYCLE_COUNT - start) > 72000U) return 0;
    }
    return 1;
}

/* 立即锁存并关断；可从异常和高优先级中断调用，不能被后续 write/enable 撤销。 */
void BSP_MotorStop(void)
{
    stopped = 1;
    TIM1->BDTR &= (uint16_t)~TIM_BDTR_MOE;
    GPIOB->BRR = MOTOR_EN | MOTOR_CAL;
    TIM1->CR1 &= (uint16_t)~TIM_CR1_CEN;
    TIM1->DIER = 0;
    ADC1->CR1 &= (uint32_t)~ADC_CR1_JEOCIE;
    TIM2->CR1 &= (uint16_t)~TIM_CR1_CEN;
    TIM2->DIER = 0;
}

/* 配置 72 MHz TIM1 中心对齐 PWM、500 ns 外加死区、低有效刹车。
 * ADC1/2 首位同时采 A/B；第二位同时采 VBUS/CBUS，12 MHz ADC 时钟。
 * OC4REF 在上数距顶点 2 us 处上升，每完整 PWM 周期仅触发一次。
 * 三相 duty 限制 0.30~0.70：采样前低侧至少已导通约 5 us。
 */
int BSP_MotorInit(uint32_t pwm_hz, MotorIoLimits *limits)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;
    TIM_BDTRInitTypeDef bdtr;
    ADC_InitTypeDef adc;
    NVIC_InitTypeDef irq;
    RCC_ClocksTypeDef clocks;
    uint16_t period;
    unsigned i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIOB->BRR = MOTOR_EN | MOTOR_CAL;
    gpio.GPIO_Pin = MOTOR_EN | MOTOR_CAL;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio);
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM2);
    BSP_MotorStop();
    RCC_GetClocksFreq(&clocks);
    if (clocks.SYSCLK_Frequency != 72000000U || clocks.HCLK_Frequency != 72000000U ||
        clocks.PCLK1_Frequency != 36000000U || clocks.PCLK2_Frequency != 72000000U ||
        pwm_hz != 20000U) return 0; /* 当前只验证该频率的低侧采样窗口。 */
    period = (uint16_t)(72000000U / (2U * pwm_hz));
    frame_cycles = 2U * period;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    CPU_CYCLE_COUNT = 0;
    CPU_CYCLE_CTRL |= 1U;
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, DISABLE);

    /* 输出预置低电平再改复用；MOE=0，OSSI/OIS 确保六路静态低。 */
    for (i = 0; i < 3; ++i) {
        GPIOA->BRR = phase_pins[i].high_pin;
        GPIOB->BRR = phase_pins[i].low_pin;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        gpio.GPIO_Pin = phase_pins[i].high_pin;
        GPIO_Init(GPIOA, &gpio);
        gpio.GPIO_Pin = phase_pins[i].low_pin;
        GPIO_Init(GPIOB, &gpio);
    }
    gpio.GPIO_Pin = MOTOR_FAULT;
    gpio.GPIO_Mode = GPIO_Mode_IPD; /* 断线下拉为故障；正常由板载 4.7k 上拉。 */
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Period = period;
    timer.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    timer.TIM_RepetitionCounter = 1; /* UG 后首个有效更新在谷底，三相同时更新。 */
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_Pulse = period / 2U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = period - 144U;
    TIM_OC4Init(TIM1, &oc);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
    TIM_BDTRStructInit(&bdtr);
    bdtr.TIM_OSSRState = TIM_OSSRState_Enable;
    bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
    bdtr.TIM_DeadTime = 36U; /* 36 / 72 MHz = 500 ns；非驱动板规格值。 */
    bdtr.TIM_Break = TIM_Break_Enable;
    bdtr.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &bdtr);
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update | TIM_FLAG_Break);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_InjecSimult;
    adc.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &adc);
    ADC_Init(ADC2, &adc);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedSequencerLengthConfig(ADC2, 2);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_13Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_71Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_5, 2, ADC_SampleTime_71Cycles5);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    /* 双注入模式由 ADC1 主触发，ADC2 不单独接受外部触发。 */
    ADC_ExternalTrigInjectedConvCmd(ADC2, DISABLE);
    if (!CalibrateAdc(ADC1) || !CalibrateAdc(ADC2)) return 0;

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 71U;
    timer.TIM_Period = 199U; /* 200 us 没完成采样处理，独立高优先级中断关断。 */
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    irq.NVIC_IRQChannelCmd = ENABLE;
    irq.NVIC_IRQChannelPreemptionPriority = 0;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannel = TIM1_BRK_IRQn;
    NVIC_Init(&irq);
    irq.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&irq);
    irq.NVIC_IRQChannelPreemptionPriority = 1;
    irq.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_Init(&irq);
    have_sample = 0;
    stopped = 0;
    limits->period_count = period;
    limits->adc_max_count = 4095U;
    limits->duty_min = 0.30f;
    limits->duty_max = 0.70f;
    return 1;
}

/* 输出关闭下唤醒 1 ms，CAL 高 200 us 后恢复增益，再等 200 us。
 * CAL 高期间放大器增益会改变，故软件零点采样必须在 CAL 低后进行。
 */
int BSP_MotorWake(void)
{
    if (stopped) return 0;
    GPIOB->BSRR = MOTOR_EN;
    WaitUs(POWER_BOARD_WAKEUP_US);
    if (!(GPIOB->IDR & MOTOR_FAULT)) { BSP_MotorStop(); return 0; }
    GPIOB->BSRR = MOTOR_CAL;
    WaitUs(200);
    GPIOB->BRR = MOTOR_CAL;
    WaitUs(200);
    if (!(GPIOB->IDR & MOTOR_FAULT)) { BSP_MotorStop(); return 0; }
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    return 1;
}

/* 唤醒和业务初始化后启动。此时 MOE 仍为 0，只产生内部采样时基。 */
void BSP_MotorSampleStart(void)
{
    if (stopped) return;
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/* 保存同时采样结果并验证节拍；ADC2 两位采样时间与 ADC1 相同。 */
int BSP_MotorRead(MotorIoSample *sample)
{
    uint32_t now = CPU_CYCLE_COUNT;
    irq_start_cycles = now;
    if (stopped || !(GPIOB->IDR & MOTOR_FAULT) || (TIM1->SR & TIM_SR_BIF) ||
        !(ADC1->SR & ADC_SR_JEOC) || !(ADC2->SR & ADC_SR_JEOC)) return 0;
    if (have_sample && (uint32_t)(now - last_sample_cycles) > frame_cycles + frame_cycles / 2U) return 0;
    last_sample_cycles = now;
    have_sample = 1;
    sample->phase_a = (uint16_t)ADC1->JDR1;
    sample->phase_b = (uint16_t)ADC2->JDR1;
    sample->bus_voltage = (uint16_t)ADC1->JDR2;
    sample->bus_current = (uint16_t)ADC2->JDR2;
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
    return 1;
}

/* 暂停更新事件后写三个预装载值，避免谷底恰好夹在三次写之间。 */
int BSP_MotorWrite(uint16_t a, uint16_t b, uint16_t c)
{
    if (stopped) return 0;
    TIM1->CR1 |= TIM_CR1_UDIS;
    *phase_pins[0].compare = a;
    *phase_pins[1].compare = b;
    *phase_pins[2].compare = c;
    TIM1->CR1 &= (uint16_t)~TIM_CR1_UDIS;
    return !stopped;
}

/* 原子检查锁存与故障后开 MOE；外部 BKIN 可不依赖 CPU 立即撤销 MOE。 */
int BSP_MotorEnable(void)
{
    uint32_t mask = __get_PRIMASK();
    int ok = 0;
    __disable_irq();
    if (!stopped && (GPIOB->IDR & MOTOR_FAULT) && !(TIM1->SR & TIM_SR_BIF)) {
        TIM1->BDTR |= TIM_BDTR_MOE;
        ok = (TIM1->BDTR & TIM_BDTR_MOE) != 0;
    }
    __set_PRIMASK(mask);
    return ok;
}

/* 处理不得跨下一采样；超出 80% 周期或出现新完成帧即关断，不追赶旧数据。 */
int BSP_MotorFinish(void)
{
    if (stopped || (uint32_t)(CPU_CYCLE_COUNT - irq_start_cycles) > frame_cycles * 4U / 5U ||
        (ADC1->SR & ADC_SR_JEOC)) return 0;
    TIM2->CNT = 0;
    return 1;
}

/* 高优先级刹车入口，不自动恢复输出。 */
int BSP_MotorBreak(void)
{
    if (!(TIM1->SR & TIM_SR_BIF)) return 0;
    BSP_MotorStop();
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    return 1;
}

/* ADC/控制停止或长时间占用 CPU 时由 TIM2 抢占并关断。 */
int BSP_MotorTimeout(void)
{
    if (!(TIM2->SR & TIM_SR_UIF)) return 0;
    BSP_MotorStop();
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    return 1;
}
