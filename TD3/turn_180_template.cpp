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
float WHEEL_BASE = 0.19f; //m

float PWM_FREQ_HZ = 20000.0f; // 20 kHz
float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS; // ~0.2475 m 

float COUNTS_PER_WHEEL_REV = PPR * 4;
float COUNTS_PER_M = COUNTS_PER_WHEEL_REV/WHEEL_CIRC;

float COUNTS_180 = COUNTS_PER_M*(PI*WHEEL_BASE/2);

volatile bool emergency_stop = false;

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

/*--------------------------BLE--------------------------*/
Serial hm10(PA_11, PA_12);   // PA_11=TX, PA_12=RX
Serial pc(USBTX, USBRX);
//DigitalOut LED(PA_5); // onboard led ld3
void serial_config(); 

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
    emergency_stop = false;  // reset flag

    stop_motors();
    wait_ms(50);

    left_encoder.reset();
    right_encoder.reset();
    last_tick_left = 0;
    last_tick_right = 0;

    float pwm_left = 0.35f;
    float pwm_right = 0.65f;

    while(!emergency_stop){

        int leftCounts = abs(left_encoder.getPulses());
        int rightCounts = abs(right_encoder.getPulses());

        int avg = (leftCounts + rightCounts)/2;

        pwmL.write(pwm_left);
        pwmR.write(pwm_right);

        if(avg >= COUNTS_180){
            break;
        }
    }

    stop_motors();
}


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
    wait_ms(300);

    left_encoder.reset();
    right_encoder.reset();

    speedTicker.attach(&speed_tick, SAMPLE_TIME);

    char s, w;

    pc.printf("Ready\r\n");
    

    while (1) {
        pc.printf("Speed L: %.3f m/s | Speed R: %.3f m/s\r\n", vL, vR);        
        if (hm10.readable()) {
            s = hm10.getc();
            pc.printf("Received: %c (%d)\r\n", s, s);
            pc.putc(s);
            
                if (s == '1') {
                    pc.printf("\r\nTurning 180...\r\n");
                    wait_ms(500);
                    turn_180();
                    pc.printf("Done\r\n");
                }
                else if (s == '0') {
                    emergency_stop = true;
                    stop_motors();
                    pc.printf("EMERGENCY STOP!\r\n");
                }
            }
        

        // PC to HM-10 (Configuration Mode)
        if (pc.readable()) {
                w = pc.getc();
                hm10.putc(w);
            }
        

        wait_ms(2);
    }
}
