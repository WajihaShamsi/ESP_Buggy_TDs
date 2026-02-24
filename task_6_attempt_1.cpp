/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "QEI.h"


/*--------------------------Constants--------------------------*/

int PPR = 256;
int GEAR_RATIO = 15;
float WHEEL_RADIUS = 0.0394f; // m
float PI = 3.1415926f;
float PWM_FREQ_HZ = 20000.0f; // 20 kHz
int COUNTS_PER_MOTOR_REV = PPR * 4;
int COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
float WHEEL_CIRC = 2.0f * PI * WHEEL_RADIUS; // ~0.2475 m
float COUNTS_PER_M = (float)COUNTS_PER_WHEEL_REV / WHEEL_CIRC;

//const float CENTER   = 0.5f;
//const float DEADBAND = 0.02f;

int COUNTS_0_5M = (int)(0.5f * COUNTS_PER_M)/19;
int COUNTS_RIGHT = (int)(0.2475f * COUNTS_PER_M)/11;

int COUNTS_180 = (int)(0.2475f *2* COUNTS_PER_M)/10;

/*--------------------------Hardware--------------------------*/
QEI left_encoder(PB_2, PB_1, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_15, PB_14, NC, PPR, QEI::X4_ENCODING);


Ticker speedTicker;
//DigitalOut led(D5); //red led 

//initialise
volatile int last_tick_left = 0, last_tick_right = 0; //previous encoder counts 

PwmOut pwmL(PC_8);
PwmOut pwmR(PC_6);

DigitalOut enable(PC_4);

// Direction only (no bipolar control pin)
DigitalOut dirL(PA_12);
DigitalOut dirR(PB_12);

/*--------------------------Helper Functions--------------------------*/

void speed_tick(){
    const int now_left = left_encoder.getPulses();    //new encoder counts
    const int now_right = right_encoder.getPulses();   

    const int dL = now_left - last_tick_left;      //difference
    const int dR = now_right - last_tick_right;

    last_tick_left = now_left; //update
    last_tick_right = now_right;

}

void setLeftMotor(float duty) {
    pwmL.write(duty);
  //  pwmR.write(0.5f);

}

void setRightMotor(float duty) {
   // pwmL.write(0.5f);
    pwmR.write(duty);
}

void stopMotors() {
    pwmL.write(0.5f);
    pwmR.write(0.5f);
}

int avgAbsTicks() {
    int l = abs(left_encoder.getPulses());
    int r = abs(right_encoder.getPulses());
    return (l + r); //add divide by 2 when the other one starts working
}

/* Drive forward for a target encoder count
void driveForwardTicks(float duty, int target_ticks) {
    left_encoder.reset();
    right_encoder.reset();

    setLeftMotor(duty);
    setRightMotor(duty);

    while (true) {
        if (avgAbsTicks() >= target_ticks) break;
    }

    stopMotors();
}

// Pivot right: left wheel moves, right wheel stopped
void turnRightPivotTicks(float duty, int target_ticks) {
    left_encoder.reset();
    right_encoder.reset();

    setLeftMotor(duty);
    setRightMotor(0.5f);

    while (true) {
        int ticks = abs(left_encoder.getPulses());
        if (ticks >= target_ticks) break;
    }

    stopMotors();
}
*/
/*--------------------------FSM--------------------------*/
enum State {
    START,
    FWD_SIDE,
    TURN_LEFT_90,
    FWD_STOP_AT_START,
    TURN_180,
    REV_SIDE,
    //TURN_LEFT_90_REV,
    DONE
};

State state = START;

int side_count = 0;       // 0..3 for forward square
int rev_side_count = 0;   // 0..3 for reverse square

/*--------------------------Main--------------------------*/

int main()
{
        // PWM frequency
    pwmL.period(1.0f / PWM_FREQ_HZ);
    pwmR.period(1.0f / PWM_FREQ_HZ);

    // bipolar
    dirL = 1;
    dirR = 1;

    enable = 1;

    left_encoder.reset();
    right_encoder.reset();

    while (true) {

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

                if (avgAbsTicks() >= COUNTS_0_5M) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    state = TURN_LEFT_90;
                }
                break;
            }

            case TURN_LEFT_90: {
                // Turn left in place: left backward, right forward
                setLeftMotor(0.5f);
                setRightMotor(0.35f);

                // Use average ticks so it works even if one wheel slips a bit
                if (avgAbsTicks() >= COUNTS_RIGHT) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();

                    side_count++;

                    if (side_count >= 4) {
                        // Finished forward square: we should be back near start
                        state = FWD_STOP_AT_START;
                    } else {
                        state = FWD_SIDE;
                    }
                }
                break;
            }

            case FWD_STOP_AT_START: {
                // Just a clean stop state (already stopped)
                stopMotors();
                left_encoder.reset();
                right_encoder.reset();
                state = TURN_180;
                break;
            }

            // ---------- Turn around ----------
            case TURN_180: {
                // Turn 180: same as two 90s
                setLeftMotor(0.5f);
                setRightMotor(0.35f);

                if (avgAbsTicks() >= COUNTS_180) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    state = REV_SIDE;
                }
                break;
            }

            // ---------- Reverse square ----------
            case REV_SIDE: {
                // Reverse direction (drive backward)
                setLeftMotor(0.35f);
                setRightMotor(0.35f);

                if (avgAbsTicks() >= COUNTS_0_5M) {
                    stopMotors();
                    left_encoder.reset();
                    right_encoder.reset();
                    state = DONE;
                }
                break;
            }
/*
            case TURN_LEFT_90_REV: {
                // While reversing square, keep same “turn left” relative to chassis:
                // still rotate the chassis left using same in-place turn
                setLeftMotor(0.35f);
                setRightMotor(0.5f);

                if (avgAbsTicks() >= TURN_90_TICKS) {
                    stopMotors();
                    ThisThread::sleep_for(150ms);
                    left_encoder.reset();
                    right_encoder.reset();

                    rev_side_count++;

                    if (rev_side_count >= 4) {
                        state = DONE;
                    } else {
                        state = REV_SIDE;
                    }
                }
                break;
            }
*/
            case DONE: {
                stopMotors();
                while (true) { }
            }
        }
    }
}
