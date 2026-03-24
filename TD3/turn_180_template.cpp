#include "mbed.h"
#include "QEI.h"
#include <cmath>
#include <cstring>


/*--------------------------Constants--------------------------*/
float SAMPLE_TIME = 0.05; //s (20 Hz) delta t
int PPR = 256;
int GEAR_RATIO = 15;

float WHEEL_RADIUS = 0.0394f; // m
float PI = 3.1415926f;
float WHEEL_BASE = 0.175f; //m

float PWM_FREQ_HZ = 20000.0f; // 20 kHz
float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS; // ~0.2475 m 

float COUNTS_PER_WHEEL_REV = PPR * 4;
float COUNTS_PER_M = COUNTS_PER_WHEEL_REV/WHEEL_CIRC;

float COUNTS_180 = COUNTS_PER_M*(PI*WHEEL_BASE/2);

/*--------------------------Velocity Globals--------------------------*/

int dL_ticks = 0;
int dR_ticks = 0;

float tickRateL = 0.0f;   // ticks/s
float tickRateR = 0.0f;

float vL = 0.0f;          // m/s (left wheel linear velocity)
float vR = 0.0f;          // m/s (right wheel linear velocity)


/*--------------------------Hardware--------------------------*/
QEI left_encoder(PB_14, PB_15, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_2, PB_1, NC, PPR, QEI::X4_ENCODING);


Ticker speedTicker;

//initialise
volatile int last_tick_left = 0, last_tick_right = 0; //previous encoder counts 

PwmOut pwmL(PC_8); // PWM output
PwmOut pwmR(PC_6);

DigitalOut enable(PC_4);

// Direction only (no bipolar control pin)
DigitalOut dirL(PA_13); // 这两个一直设成1就行了
DigitalOut dirR(PB_12);

/*AnalogIn pot1(A0);
AnalogIn pot2(A1);*/

static inline float tickRateToVel(float tick_rate_ticks_per_s, float counts_per_m) {
    return tick_rate_ticks_per_s / counts_per_m; // 这已经算出来是speed了
}
void speed_tick() {
    const int now_left  = left_encoder.getPulses();
    const int now_right = right_encoder.getPulses();

    dL_ticks = now_left  - last_tick_left;
    dR_ticks = now_right - last_tick_right;

    last_tick_left  = now_left;
    last_tick_right = now_right;

    tickRateL = (float)dL_ticks / SAMPLE_TIME; // ticks/s
    tickRateR = (float)dR_ticks / SAMPLE_TIME;

    vL = tickRateToVel(tickRateL, COUNTS_PER_M); // m/s
    vR = tickRateToVel(tickRateR, COUNTS_PER_M);

}

void stop_motors(){
    pwmL.write(0.5f);
    pwmR.write(0.5f);
}

void turn_180(){
    stop_motors();
    ThisThread::sleep_for(50ms);
    // reset encoders so turn starts from zero
    left_encoder.reset();
    right_encoder.reset();
    last_tick_left = 0;
    last_tick_right = 0;

    float pwm_left = 0.35f;
    float pwm_right = 0.65f;

    while(true){
        int leftCounts = left_encoder.getPulses();
        int rightCounts = right_encoder.getPulses();

        int avg = (leftCounts + rightCounts)/2;

        pwmL.write(pwm_left);
        pwmR.write(pwm_right);

        if(avg >= COUNTS_180){
            stop_motors();
        }
    }

}
/*--------------------------BLE--------------------------*/
UnbufferedSerial hm10(PA_11, PA_12);   // PA_11=TX, PA_12=RX
UnbufferedSerial pc(USBTX, USBRX);
//DigitalOut LED(PA_5); // onboard led ld3


int main() {
    pc.baud(9600);
    hm10.baud(9600);

    pwmL.period(1.0f / PWM_FREQ_HZ);
    pwmR.period(1.0f / PWM_FREQ_HZ);

    enable = 1;

    // default idle directions
    dirL = 1;
    dirR = 1;

    stop_motors();
    ThisThread::sleep_for(300ms);

    left_encoder.reset();
    right_encoder.reset();

    speedTicker.attach(&speed_tick, SAMPLE_TIME);

    char s, w;
    const char *ready = "Ready\r\n";
    pc.write(ready, sizeof("Ready\r\n") - 1);

    while (1) {
  
        if (hm10.readable()) {
            if (hm10.read(&s, 1) == 1) {
                pc.write(&s, 1);

                if (s == '1') {
                    const char *msg = "\r\nTurning 180...\r\n";
                    pc.write(msg, sizeof("Turning 180\r\n") - 1);
                    turn_180();
                    const char *done = "Done\r\n";
                    pc.write(done, sizeof("Done\r\n") - 1);
                }
                else if (s == '0') {
                    stop_motors();
                }
            }
        }

        // PC to HM-10 (Configuration Mode)
        if (pc.readable()) {
            if (pc.read(&w, 1) == 1) {
                hm10.write(&w, 1);
            }
        }

        ThisThread::sleep_for(2ms);
    }
}
