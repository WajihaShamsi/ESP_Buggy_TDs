#include "mbed.h"

// ======================
// 需要你确认/修改的引脚
// ======================
// PWM 输出 -> 接 Motor Drive Board JP1a: PWM1 (pin 3), PWM2 (pin 6)
static const PinName PWM1_PIN = D9;    // MCU -> PWM1
static const PinName PWM2_PIN = D10;   // MCU -> PWM2

// Enable 输出 -> 接 Motor Drive Board JP1a: Enable (pin 7)
static const PinName EN_PIN   = D8;    // MCU -> Enable

// 电位器输入（你的 mbed application board 上的 Pot1/Pot2 通常接到 A0/A1）
static const PinName POT1_PIN = A0;    // Pot1 -> duty for PWM1
static const PinName POT2_PIN = A1;    // Pot2 -> duty for PWM2

// ======================
// 参数：建议值
// ======================
static const float PWM_FREQ_HZ = 20000.0f;  // 20 kHz：避开可听啸叫
static const float DUTY_MIN    = 0.05f;     // 避免 0%/100% 边界行为
static const float DUTY_MAX    = 0.95f;

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int main() {
    // ---- 输出/输入对象 ----
    PwmOut pwm1(PWM1_PIN);
    PwmOut pwm2(PWM2_PIN);
    DigitalOut enable(EN_PIN, 0);   // 默认先关，避免上电瞬间乱动
    AnalogIn pot1(POT1_PIN);
    AnalogIn pot2(POT2_PIN);

    // ---- 可选：串口打印 ----
    /*BufferedSerial pc(USBTX, USBRX, 115200);
    pc.set_format(8, BufferedSerial::None, 1);*/

    // ---- 设置 PWM 频率（周期）----
    const float period_s = 1.0f / PWM_FREQ_HZ;
    pwm1.period(period_s);
    pwm2.period(period_s);

    // ---- 初值 ----
    pwm1.write(0.30f);
    pwm2.write(0.70f);

    // ---- 使能驱动板 ----
    enable = 1;

    // 给电路一点稳定时间（可选）
    /*ThisThread::sleep_for(50ms);*/

    // ---- 主循环：实时更新 duty ----
    int print_div = 0;
    while (true) {
        // 读电位器（0.0~1.0）
        float duty1 = pot1.read();
        float duty2 = pot2.read();

        // 限幅（5%~95%）
        duty1 = clampf(duty1, DUTY_MIN, DUTY_MAX);
        duty2 = clampf(duty2, DUTY_MIN, DUTY_MAX);

        // 写入 PWM（核心）
        pwm1.write(duty1);
        pwm2.write(duty2);

        // 每 200ms 打印一次（可选）
        /*if (++print_div >= 10) { // 10 * 20ms = 200ms
            print_div = 0;
            char buf[80];
            int n = snprintf(buf, sizeof(buf),
                             "PWM1 duty=%.2f, PWM2 duty=%.2f\r\n",
                             duty1, duty2);
            pc.write(buf, n);
        }

        // 20ms 更新一次（足够“run-time control”，且稳定）
        ThisThread::sleep_for(20ms);*/
    }
}