#include "mbed.h"
#include "QEI.h"
#include <cmath>
#include "Serial.h"
Serial pc(USBTX, USBRX);
Serial hm10(PA_11, PA_12);   // PA_11=TX, PA_12=RX


/*======================== SENSOR HARDWARE ========================*/
AnalogIn sensors[6] = {A0, A1, A2, A3, A4, A5};
DigitalOut darlington[6]={D8,D7,D6,D5,D4,D3};
float sensor_values[6];

// ── Hardcode your measured black/white values here ──
float min_vals[6] = {0.092f, 0.086f, 0.104f, 0.105f, 0.101f, 0.083f};  // black
float max_vals[6] = {0.918f, 0.875f, 0.916F, 0.920f, 0.920f, 0.806f};  // white
float range[6]   = {0};

/*======================== LINE PID GAINS ========================*/
float line_Kp = 0.04f;
float line_Ki = 0.0f;
float line_Kd = 0.08f;

float base_speed_ms = 0.25f;

/*======================== LINE PID STATE ========================*/
float error_val  = 0.0f;
float last_error = 0.0f;
float integral   = 0.0f;
float derivative = 0.0f;
float correction = 0.0f;

volatile float target_speed_L = 0.0f;
volatile float target_speed_R = 0.0f;

int lost_counter = 0;

/*======================== SPEED CONSTANTS ========================*/
float SAMPLE_TIME   = 0.01f;
int   PPR           = 256;
int   GEAR_RATIO    = 15;
float WHEEL_RADIUS  = 0.0394f;
float PI            = 3.1415926f;
float WHEEL_BASE    = 0.175f;
float PWM_FREQ_HZ   = 20000.0f;
float WHEEL_CIRC    = 2.0f * 3.1415926f * 0.0394f;
float COUNTS_PER_WHEEL_REV = 256 * 4;
float COUNTS_PER_M  = (256 * 4) / (2.0f * 3.1415926f * 0.0394f);

/* 180 DEGREE TURN (ENCODER BASED) */
float COUNTS_180 = (COUNTS_PER_M * (PI * WHEEL_BASE/2));


float Kp_L = 0.08f;
float Kp_R = 0.08f;
float Ki   = 0.0f;
float Kd   = 0.04f;
/*======================== EMERGENCY ========================*/
volatile bool emergency_stop = false;

/*======================== VELOCITY GLOBALS ========================*/
volatile int   dL_ticks = 0,  dR_ticks = 0;
volatile float tickRateL = 0.0f, tickRateR = 0.0f;
volatile float vL = 0.0f,  vR = 0.0f;
volatile float uL = 0.5f,  uR = 0.5f;
volatile float eL = 0.0f,  eR = 0.0f;
volatile float uL_prev = 0.5f, uR_prev = 0.5f;
volatile float eL_prev = 0.0f, eR_prev = 0.0f;
volatile float eL_prev2 = 0.0f, eR_prev2 = 0.0f;
volatile float v_robot = 0.0f, w_robot = 0.0f;

/*======================== HARDWARE ========================*/
QEI left_encoder(PB_14, PB_15, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_2,  PB_1,  NC, PPR, QEI::X4_ENCODING);

PwmOut pwmL(PC_8);
PwmOut pwmR(PC_6);

DigitalOut enable(PC_4);
DigitalOut dirL(PA_13);
DigitalOut dirR(PB_12);

Ticker speedTicker;
Ticker line_ticker;

volatile int last_tick_left = 0, last_tick_right = 0;

/*======================== HELPERS ========================*/
float EnsureSafe(float x, float min_val, float max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static inline float tickRateToVel(float tick_rate, float counts_per_m) {
    return tick_rate / counts_per_m;
}

/*======================== MOTOR STOP ========================*/
void stop_motors() {
    pwmL.write(0.5f);
    pwmR.write(0.5f);
}

/*======================== FSM ========================*/
enum State { FOLLOWING, LOST, STOPPED, TURNING };
State current_state = FOLLOWING;

/*======================== SENSOR FUNCTIONS ========================*/
void read_sensors() {
    for (int i = 0; i < 6; i++) {
        float raw  = sensors[i].read();
        float norm = 0.0f;
        if (range[i] > 0.0001f)
            norm = (raw - min_vals[i]) / range[i];
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        sensor_values[i] = norm;
    }
}

float calculate_error() {
    float weights[6]      = {3.0f, 2.0f, 1.0f, -1.0f, -2.0f, -3.0f};
    float weighted_sum    = 0.0f;
    float total_signal    = 0.0f;
    float threshold       = 0.05f;
    bool  line_detected   = false;

    for (int i = 0; i < 6; i++) {
        float val = sensor_values[i];
        if (val > threshold) line_detected = true;
        weighted_sum += val * weights[i];
        total_signal += val;
    }

    if (!line_detected || total_signal < 0.07f) {
        current_state = LOST;
        return last_error;
    }

    current_state = FOLLOWING;
    lost_counter  = 0;

    if (total_signal > 0.0001f)
        return weighted_sum / total_signal;
    else
        return last_error;
}

/*======================== LINE PID LOOP ========================*/
void pid_control_loop() {
    read_sensors();
    error_val = calculate_error();

    if (emergency_stop || current_state == TURNING)
        return;

    switch (current_state) {
        case FOLLOWING:
            integral   += error_val;
            derivative  = error_val - last_error;
            correction  = (error_val  * line_Kp)
                        + (integral   * line_Ki)
                        + (derivative * line_Kd);
            last_error  = error_val;

            target_speed_L = base_speed_ms + correction;
            target_speed_R = base_speed_ms - correction;
            break;

        case LOST: {
            lost_counter++;
            float search_turn = 0.15f;

            if (last_error > 0) {
                target_speed_L = base_speed_ms - search_turn;
                target_speed_R = base_speed_ms + search_turn;
            } else {
                target_speed_L = base_speed_ms + search_turn;
                target_speed_R = base_speed_ms - search_turn;
            }

            if (lost_counter > 150) current_state = STOPPED;
            break;
        }

        case STOPPED:
            target_speed_L = 0.0f;
            target_speed_R = 0.0f;
            break;

        default:
            break;
    }
}

/*======================== SPEED TICK ========================*/
void speed_tick() {
    if (emergency_stop || current_state == STOPPED || current_state == TURNING) {
        stop_motors();
        return;
    }

    const int now_left  = left_encoder.getPulses();
    const int now_right = right_encoder.getPulses();

    dL_ticks = now_left  - last_tick_left;
    dR_ticks = now_right - last_tick_right;

    last_tick_left  = now_left;
    last_tick_right = now_right;

    tickRateL = (float)dL_ticks / SAMPLE_TIME;
    tickRateR = (float)dR_ticks / SAMPLE_TIME;

    vL = tickRateToVel(tickRateL, COUNTS_PER_M);
    vR = tickRateToVel(tickRateR, COUNTS_PER_M);

    eL = target_speed_L - vL;
    eR = target_speed_R - vR;

    uL = uL_prev - (eL * Kp_L)-(eL - eL_prev) * Kd; 
    uR = uR_prev - (eR * Kp_R)-(eR - eR_prev) * Kd;

    uL = EnsureSafe(uL, 0.0f, 1.0f);
    uR = EnsureSafe(uR, 0.0f, 1.0f);

    v_robot = 0.5f * (vR + vL);
    w_robot = (vR - vL) / WHEEL_BASE;

    pwmL.write(uL);
    pwmR.write(uR);

    uL_prev = uL;
    uR_prev = uR;

    eL_prev2 = eL_prev;
    eR_prev2 = eR_prev;
    eL_prev  = eL;
    eR_prev  = eR;
}

/*======================== TURN 180 (FIXED) ========================*/
void turn_180() {

    emergency_stop = false;
    current_state = TURNING;

    stop_motors();
    wait_ms(50);

    left_encoder.reset();
    right_encoder.reset();
    last_tick_left = 0;
    last_tick_right = 0;

    float pwm_left = 0.35f;
    float pwm_right = 0.65f;

    while (!emergency_stop) {

        int leftCounts = abs(left_encoder.getPulses());
        int rightCounts = abs(right_encoder.getPulses());

        int avg = (leftCounts + rightCounts) / 2;

        pwmL.write(pwm_left);
        pwmR.write(pwm_right);

        if (avg >= COUNTS_180*1.05)
            break;
    }

    stop_motors();

    uL_prev = 0.5f;
    uR_prev = 0.5f;
    eL_prev = eR_prev = 0.0f;
    eL_prev2 = eR_prev2 = 0.0f;
    integral = 0.0f;
    last_error = 0.0f;

    current_state = FOLLOWING;
}



/*======================== MAIN ========================*/
int main() {
    pc.baud(9600);
    hm10.baud(9600);

    // Compute ranges from hardcoded min/max
    for (int i = 0; i < 6; i++) {
        range[i] = max_vals[i] - min_vals[i];
    }

    // PWM setup
    pwmL.period(1.0f / PWM_FREQ_HZ);
    pwmR.period(1.0f / PWM_FREQ_HZ);

    dirL = 1;
    dirR = 1;
    enable = 1;

    left_encoder.reset();
    right_encoder.reset();

    uL_prev = 0.5f;
    uR_prev = 0.5f;
    pwmL.write(0.5f);
    pwmR.write(0.5f);

    for (int i = 0; i < 6; i++) {
        darlington[i] = 1;
    }

    line_ticker.attach(&pid_control_loop, 0.01f);

    wait_ms(2000);
    speedTicker.attach(&speed_tick, SAMPLE_TIME);

    char s;

    while (true) {
        if (hm10.readable()) {
            s = hm10.getc();
            pc.printf("BLE: %c\r\n", s);

            if (s == '1') {
                emergency_stop = false;
                current_state = FOLLOWING;
            }

            else if (s == '2') {
                turn_180();
            }

            else if (s == '0') {
                emergency_stop = true;
                current_state = STOPPED;
                stop_motors();
            }
        }
    }
}
