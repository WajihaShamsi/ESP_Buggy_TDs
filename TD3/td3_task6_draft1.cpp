#include "mbed.h"
#include "QEI.h"

const int PPR = 256;
float WHEEL_RADIUS   = 0.0394f;
float PI             = 3.14159f;
float COUNTS_PER_M   = (PPR * 4) / (2.0f * PI * WHEEL_RADIUS);

QEI left_encoder(PB_2, PB_1, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_15, PB_14, NC, PPR, QEI::X4_ENCODING);

PwmOut pwmL(PC_8);
PwmOut pwmR(PC_6);

DigitalOut enable(PC_4);
DigitalOut dirL(PA_12);
DigitalOut dirR(PB_12);

AnalogIn sensors[6] = {A0, A1, A2, A3, A4, A5}; 
DigitalOut ir_pwr(D2); // Pin that turns the sensor LEDs on/off-- incase not using darlington array- not needed?

float STOP_DISTANCE_MM = 200.0f;
float BRAKE_SPEED  = 0.4f; //djust as necessary
float MIN_PWM      = 0.1f;

float clean_values[6];
int   snap_eol_L    = 0; //snaphsot of tick values when eol is triggered 
int   snap_eol_R    = 0;
int eol_count = 0;
const int eol_confirm = 1; // using 20 hz jsut one is fine else use 4/5 if at 50 hz


void clean_sensor_reading() {
    float ambient[6], active[6];
    ir_pwr = 0;
    wait_us(50);
    for(int i = 0; i < 6; i++) ambient[i] = sensors[i].read();
    ir_pwr = 1;
    wait_us(50);
    for(int i = 0; i < 6; i++) active[i] = sensors[i].read();
    for(int i = 0; i < 6; i++) {
        clean_values[i] = active[i] - ambient[i];
        if(clean_values[i] < 0) clean_values[i] = 0;
    }
}
/* depends on darlingtonm array. incase not using darlingron array, use:
void clean_sensor_reading() {
    for(int i = 0; i < 6; i++) {
        clean_values[i] = sensors[i].read();
    }
}*/

void Braking();
void stopMotor();
void run_following_test();
bool end_of_line_detected();
void control_loop();

typedef enum {
    FOLLOWING,
    BRAKING,
    STOPPED
} State;

State state = FOLLOWING;

Ticker ticker;

int main(){
    pwmL.period(1.0f / 20000.0f); //set freq to 20 kHz
    pwmR.period(1.0f / 20000.0f);

    dirL = 1;
    dirR = 1;

    enable = 1;

    ticker.attach(&control_loop, 50ms);

    while(true) {
        ThisThread::sleep_for(500ms);
    }

}


void control_loop() {
    clean_sensor_reading(); // depends again on dar array

    switch(state) {
        case FOLLOWING:
            if (end_of_line_detected()) {
                snap_eol_L = left_encoder.getPulses();
                snap_eol_R = right_encoder.getPulses();
                state = BRAKING;
            } else {
                run_following_test(); // runs in stright line, in integrated program to be replaced with actual pid logic i.e. pid_control_loop()
            }
            break;

        case BRAKING:
            Braking();
            break;

        case STOPPED:
            stopMotor();
            break;
    }
}
void stopMotor(){
    pwmL.write(0.5);
    pwmR.write(0.5);
}

void run_following_test() {
    pwmL.write(0.5f - BRAKE_SPEED);  // for purely testing purposes, run in straight line
    pwmR.write(0.5f - BRAKE_SPEED);
}

bool end_of_line_detected() {
    float sum = 0;
    for (int i = 0; i < 6; i++){
        sum = sum + clean_values[i];
    }

    bool all_dark = (sum < 0.1f);
    if (all_dark) {
        eol_count++;
    }
    else {
        eol_count = 0;
    }
return (eol_count >= eol_confirm);
}

void Braking() {
    int dL = left_encoder.getPulses() - snap_eol_L;
    int dR = right_encoder.getPulses() - snap_eol_R;
    float travelled_mm = ((dL + dR) / 2.0f) / (COUNTS_PER_M / 1000.0f);
    float remaining   = STOP_DISTANCE_MM - travelled_mm;


    if (remaining <= 0.0f) {
        pwmL.write(0.5f);
        pwmR.write(0.5f);
        state = STOPPED;
        return;
    }

/*  Linear ramp down: speed decreases from BRAKE_SPEED to MIN_PWM as remaining distance approaches 0
t = 1.0 at eol trigger, t = 0.0 at target stop point
This prevents a sudden jerky braking by smoothly reducing speed over the 200mm braking distance
*/
    float t = remaining / STOP_DISTANCE_MM;
    float speed = MIN_PWM + t * (BRAKE_SPEED - MIN_PWM);

    pwmL.write(0.5f - speed);
    pwmR.write(0.5f - speed);
}

/* void pid_control_loop() {
    clean_sensor_reading(); 
    error_val = calculate_error(); 

    switch(current_state) {
        case FOLLOWING:
            greenLED.on();
            blueLED.off();
            redLED.off();

            integral += error_val;
            derivative = error_val - last_error;
            correction = (error_val * Kp) + (integral * Ki) + (derivative * Kd);
            last_error = error_val;
            
            target_speed_L = base_speed + correction;
            target_speed_R = base_speed - correction;
            break;

        case LOST:
            // LED Indication: Blue for searching/lost
            greenLED.off();
            blueLED.on();
            redLED.off();

            lost_counter++;
            if (lost_counter <= 3) {
                target_speed_L = base_speed + correction;
                target_speed_R = base_speed - correction;
            } 
            else {
                //flip the correction sign and slow down to 60% speed 
                // Slowing down helps the sensors "catch" the line better
                target_speed_L = (base_speed * 0.6f) - correction; 
                target_speed_R = (base_speed * 0.6f) + correction;
            }
            //Give up after 2 seconds (100 cycles)
            if (lost_counter > 100) current_state = STOPPED;
            break;

        case STOPPED:
            // LED Indication: Red for emergency stop
            greenLED.off();
            blueLED.off();
            redLED.on();

            target_speed_L = 0.0f;
            target_speed_R = 0.0f;
            break;
    }
}
*/