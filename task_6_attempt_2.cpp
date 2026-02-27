/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "QEI.h"
#include "C12832.h"


/*--------------------------Constants--------------------------*/
float SAMPLE_TIME = 0.05; //s (20 Hz)
int PPR = 256;
int GEAR_RATIO = 15;

float WHEEL_RADIUS = 0.0394f; // m
float PI = 3.1415926f;
float WHEEL_BASE = 0.175f; //m

float PWM_FREQ_HZ = 20000.0f; // 20 kHz
float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS; // ~0.2475 m


//int COUNTS_PER_MOTOR_REV = PPR * 4;
//int COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
//int COUNTS_PER_M = COUNTS_PER_WHEEL_REV/WHEEL_CIRC;

float COUNTS_PER_WHEEL_REV_R = 1027.0f; //from trial and error
float COUNTS_PER_WHEEL_REV_L = 1030.0f;

float COUNTS_PER_M_R = (float)COUNTS_PER_WHEEL_REV_R / WHEEL_CIRC;
float COUNTS_PER_M_L = (float)COUNTS_PER_WHEEL_REV_L / WHEEL_CIRC;


//float COUNTS_PER_M_R = 4896.0f; //from trial and error
//float COUNTS_PER_M_L = 4906.0f;

float COUNTS = (COUNTS_PER_M_L + COUNTS_PER_M_R)/2;

/*--------------------------Hardware--------------------------*/
QEI left_encoder(PB_2, PB_1, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_15, PB_14, NC, PPR, QEI::X4_ENCODING);


Ticker speedTicker;

//initialise
volatile int last_tick_left = 0, last_tick_right = 0; //previous encoder counts 

PwmOut pwmL(PC_8);
PwmOut pwmR(PC_6);

DigitalOut enable(PC_4);

// Direction only (no bipolar control pin)
DigitalOut dirL(PA_12);
DigitalOut dirR(PB_12);

/*--------------------------Helper Functions--------------------------*/
static int distToTicks(float dist_m) {
    return (int)(dist_m * COUNTS + 0.5f);
}

// sL = ticksL / COUNTS_PER_M_L ; sR = ticksR / COUNTS_PER_M_R
// theta = (sR - sL) / WHEEL_BASE
static float estimateThetaRad() {
    const float sL = (float)left_encoder.getPulses()  / COUNTS_PER_M_L;
    const float sR = (float)right_encoder.getPulses() / COUNTS_PER_M_R;
    return abs((sR - sL) / WHEEL_BASE);
}

void speed_tick() {
    const int now_left  = left_encoder.getPulses();
    const int now_right = right_encoder.getPulses();

    const int dL = now_left  - last_tick_left;
    const int dR = now_right - last_tick_right;

    last_tick_left  = now_left;
    last_tick_right = now_right;
}


static int turnToTicks(float theta_rad, float counts) {
    float arc_m = (WHEEL_BASE * 0.5f) * theta_rad; // per wheel
    return (int)(arc_m * counts/1.3);
}

void setLeftMotor(float duty) {pwmL.write(duty);}

void setRightMotor(float duty) {pwmR.write(duty);}

void stopMotors() {
    pwmL.write(0.5f);
    pwmR.write(0.5f);
}

static float estimateDistanceM() {
    float dL = fabsf((float)left_encoder.getPulses())  / COUNTS_PER_M_L;
    float dR = fabsf((float)right_encoder.getPulses()) / COUNTS_PER_M_R;
    return 0.5f * (dL + dR);
}

int avgAbsTurnLeft() {
    int l = abs(left_encoder.getPulses());
    return l;
}

int avgAbsTurnRight() {
    int r = abs(right_encoder.getPulses());
    return r;
}

volatile bool go_done = false;
//Interrupt
void fire(){
    go_done = true;
}

/*--------------------------FSM--------------------------*/
enum State {
    START,
    FWD_SIDE,
    TURN_LEFT_90,
    FWD_STOP_AT_START,
    TURN_180,
    REV_SIDE,
    TURN_LEFT_90_REV,
    ONE_MORE,
    DONE
};

State state = START;

int side_count = 0;       
int rev_side_count = 0;   

/*--------------------------Main--------------------------*/

int main()
{
    C12832 lcd(D11, D13, D12, D7, D10); 
    InterruptIn fireJoy(D4);
    fireJoy.rise(&fire);
    speedTicker.attach(&speed_tick, SAMPLE_TIME); //20 Hz speed update

    // PWM frequency
    pwmL.period(1.0f / PWM_FREQ_HZ);
    pwmR.period(1.0f / PWM_FREQ_HZ);

    // bipolar
    dirL = 1;
    dirR = 1;

    //set enable high
    enable = 1;

    stopMotors();

    left_encoder.reset();
    right_encoder.reset();

    int COUNTS_0_5M = distToTicks(1.17f);
    float THETA_90  = 1.9f;
    float THETA_180 = 3.4;

    //reading values
    int left_encoder_read_p = 0;
    int right_encoder_read_p = 0;

    while (true) {

        if (go_done && state != DONE) {
            stopMotors();
            state = DONE;
        }

        switch (state) {

            case START: {
                side_count = 0;
                rev_side_count = 0;
                left_encoder.reset();
                right_encoder.reset();
                state = FWD_SIDE;
                break;
            }

            // ---------- Forward square ----------
            case FWD_SIDE: {
                setLeftMotor(0.35f);
                setRightMotor(0.35f);

                if (estimateDistanceM() >= 0.45f) {
                    stopMotors();

                    //read values
                    left_encoder_read_p = left_encoder.getPulses();
                    right_encoder_read_p = right_encoder.getPulses();

                    left_encoder.reset();
                    right_encoder.reset();
                    wait_ms(200);
                    state = TURN_LEFT_90;
                    //state = DONE;
                }
                break;
            }

            case TURN_LEFT_90: {
                // Turn left in place: left backward, right forward
                setLeftMotor(0.35f);
                setRightMotor(0.65f);

                // Use average ticks so it works even if one wheel slips a bit
                if (estimateThetaRad() >= THETA_90) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);

                    side_count++;

                    if (side_count >= 3) {
                        // Finished forward square: we should be back near start
                        state = FWD_STOP_AT_START;
                    } else {
                        state = FWD_SIDE;
                    }
                }
                break;
            }

            case FWD_STOP_AT_START: {
                setLeftMotor(0.35f);
                setRightMotor(0.35f);

                if (estimateDistanceM() >= 0.450f) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);
                    state = TURN_180;
                    //state = DONE;
                }
                break;
            }

            // ---------- Turn around ----------
            case TURN_180: {
                // Turn 180: same as two 90s
                setLeftMotor(0.35f);
                setRightMotor(0.65f);

                if (estimateThetaRad() >= THETA_180) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);
                    state = REV_SIDE;
                }
                break;
            }

            // ---------- Reverse square ----------
            case REV_SIDE: {
                // Reverse direction 
                setLeftMotor(0.35f);
                setRightMotor(0.35f);

                if (estimateDistanceM() >= 0.450f) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);
                    state = TURN_LEFT_90_REV;
                }
                break;
            }

            case TURN_LEFT_90_REV: {
                //rotate the chassis left using same in-place turn
                setLeftMotor(0.65f);
                setRightMotor(0.35f);

                if (estimateThetaRad() >= THETA_90) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);

                    rev_side_count++;

                    if (rev_side_count >= 3) {
                        state = ONE_MORE;
                    } else {
                        state = REV_SIDE;
                    }
                }
                break;
            }

            case ONE_MORE: {
                // Reverse direction 
                setLeftMotor(0.35f);
                setRightMotor(0.35f);

                if (estimateDistanceM() >= 0.450f) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    last_tick_left = 0;
                    last_tick_right = 0;
                    wait_ms(200);
                    state = DONE;
                }
                break;
            }

              case DONE: {
                stopMotors();
                //lcd.cls();
                //lcd.locate(0,0);
                //lcd.printf("Lpulses: %d, Rpulses: %d", left_encoder_read_p, right_encoder_read_p);
                while (true) { }
            }
        }
    }
}
