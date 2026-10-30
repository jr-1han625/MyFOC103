/* FOC 公式验收装配入口：初始化显示与计算状态，由调试器按需触发单步。 */
#include "driver_assembly.h"
#include "OLED.h"
#include "foc_bench.h"
#include "xy_select.h"

FocBench foc_bench;

/* 主循环只负责单步调度；真实硬件接入后应由同步采样完成入口调度电流环。 */
int main(void)
{
    Drivers_Bind();
    Foc_Init();
    /* 按手册提供名义标度；零点、极性未实测前保持不可用，且不使能功率输出。 */
    foc.period_s = FOC_CONTROL_PERIOD_S;
    foc.calibration.adc_max_count = FOC_ADC_MAX_COUNT;
    foc.calibration.offset_a_count = -1.0f;
    foc.calibration.offset_b_count = -1.0f;
    foc.calibration.amp_per_count_a = FOC_PHASE_AMP_PER_COUNT_MAGNITUDE * FOC_PHASE_CURRENT_POLARITY;
    foc.calibration.amp_per_count_b = FOC_PHASE_AMP_PER_COUNT_MAGNITUDE * FOC_PHASE_CURRENT_POLARITY;
    OLED_B.init();
    OLED_B.show_string(1, 1, "FOC Math Ready");
    OLED_B.show_string(2, 1, "Status:");
    OLED_B.show_num(2, 8, (uint32_t)foc.status, 1);

    while (1)
    {
        if (foc_bench.step_requested != 0)
        {
            foc_bench.step_requested = 0;
            Foc_ControlStep(&foc_bench.sample);
            OLED_B.show_num(2, 8, (uint32_t)foc.status, 1);
        }
    }
}
