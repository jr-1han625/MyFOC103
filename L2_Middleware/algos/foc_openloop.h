/* 开环电角度算子：按电频率斜坡积分相位，查表插值得到正余弦。
 * 所有状态由调用者持有；不访问外设或业务全局。此角度不是测得的转子角度。
 */
#ifndef FOC_OPENLOOP_H
#define FOC_OPENLOOP_H
/* 一套独立的频率/相位历史，调用者初始化为 0。 */
typedef struct {
    float frequency_hz; /* 当前电频率 Hz，由算子每步更新。 */
    float phase_turn; /* [0,1)，电周数，由算子积分并回绕。 */
} FocOpenLoopState;
/* 本次输出；调用者可直接传 sin/cos 给反 Park。 */
typedef struct {
    float sin_theta; /* sin(2*pi*phase_turn)。 */
    float cos_theta; /* cos(2*pi*phase_turn)。 */
} FocOpenLoopOutput;
/* target_hz >= 0，acceleration_hz_s > 0，period_s > 0；每步相位增量须小于一周。
 * 成功返回 1；非法输入返回 0 且不改变 state/output。 */
int Foc_OpenLoopStep(float target_hz, float acceleration_hz_s, float period_s,
                     FocOpenLoopState *state, FocOpenLoopOutput *output);
#endif
