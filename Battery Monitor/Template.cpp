#include "mbed.h"
#include "QEI.h"
#include <cmath>
#include "Serial.h"
#include "ds2781.h"
Serial pc(USBTX, USBRX);
Serial hm10(PA_11, PA_12);   // PA_11=TX, PA_12=RX

/*======================== SENSOR HARDWARE ========================*/
AnalogIn sensors[6] = {A0, A1, A2, A3, A4, A5};
DigitalOut darlington[6]={D8,D7,D6,D5,D4,D3};
float sensor_values[6];

// ── Hardcode  measured black/white values  ──
float min_vals[6] = {0.136f, 0.106f, 0.131f, 0.134f, 0.131f, 0.100f};  // black
float max_vals[6] = {0.932f, 0.925f, 0.930F, 0.930f, 0.928f, 0.902f};  // white
float range[6]   = {0};

/*======================== DS2781 BATTERY MONITOR ========================*/
DigitalInOut one_wire_pin(PC_12);
int VoltageReading, CurrentReading;
float Voltage, Current;

/* Battery saving control */
volatile float battery_scale = 1.0f;     // 0.0 to 1.0, scales requested speed
volatile bool low_battery = false;
volatile bool high_current = false;

/* Tunable thresholds */
const float VOLT_WARN = 7.2f;   // start reducing speed here
const float VOLT_MIN  = 6.8f;   // strong reduction here
const float CURR_WARN = 1.2f;   // start reducing speed if current too high
const float CURR_MAX  = 1.8f;   // stronger reduction
const float MIN_SPEED_SCALE = 0.45f;   // never go below 45% of base speed unless you want near-stop

/*======================== LINE PID GAINS ========================*/
float line_Kp = 0.05f;
float line_Ki = 0.0f;
float line_Kd = 0.1f;

float base_speed_ms = 0.40f;

/*======================== LINE PID STATE ========================*/
float error_val  = 0.0f;
float last_error = 0.0f;
float integral   = 0.0f;
float derivative = 0.0f;
float correction = 0.0f;

volatile float target_speed_L = 0.0f;
volatile float target_speed_R = 0.0f;

int lost_counter = 0;

/*======================== EOL LATCH SYSTEM ========================*/
int eol_counter = 0;
const int EOL_CONFIRM = 0;

bool was_centered = false;
//bool eol_active = false;

int center_counter = 0;
const int CENTER_CONFIRM = 3;

volatile int snap_eol_L = 0;
volatile int snap_eol_R = 0;

float STOP_DISTANCE_MM = 180.0f;
/*======================== SPEED CONSTANTS ========================*/
float SAMPLE_TIME   = 0.01f;
int   PPR           = 256;
int   GEAR_RATIO    = 15;
float WHEEL_RADIUS  = 0.0394f;
float PI            = 3.1415926f;
float WHEEL_BASE    = 0.19f;
float PWM_FREQ_HZ   = 20000.0f;
float WHEEL_CIRC    = 2.0f * 3.1415926f * 0.0394f;
float COUNTS_PER_WHEEL_REV = 256 * 4;
float COUNTS_PER_M  = (256 * 4) / (2.0f * 3.1415926f * 0.0394f);

/* 180 DEGREE TURN (ENCODER BASED) */
float COUNTS_180 = (COUNTS_PER_M * (PI * WHEEL_BASE/2));


float Kp_L = 0.08f;
float Kp_R = 0.08f;
float Ki   = 0.0f;
float Kd   = 0.09f;
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
enum State { FOLLOWING, LOST, STOPPED, TURNING, BRAKING };
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

/*======================== ERROR ========================*/
float calculate_error() {

    float weights[6] = {3, 2, 1, -1, -2, -3};
    float sum = 0, total = 0;
    float minv = 1, maxv = 0;

    for (int i = 0; i < 6; i++) {
        float v = sensor_values[i];
        sum += v * weights[i];
        total += v;
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
    }

    // Signal gone — decide here, no double reading
    if (total < 0.05f) {
        if (current_state == BRAKING) return last_error;  // ← ADD THIS - don't interfere
        if (was_centered) {
            // was on line → end of line
            snap_eol_L = left_encoder.getPulses();
            snap_eol_R = right_encoder.getPulses();
            current_state = BRAKING;
        } else {
            // never centred → genuinely lost
            current_state = LOST;
            lost_counter  = 0;
        }
        return last_error;
    }

    // Signal present — update was_centered and return error
    float error = sum / total;

    if (fabs(error) < 0.4f) {
        was_centered = true;
    } else if (fabs(error) > 0.8f) {
        was_centered = false;
    }

    return error;
}

//braking function 
void do_braking() {

    // Compute distance travelled since EOL snap
    int dL = left_encoder.getPulses() - snap_eol_L;
    int dR = right_encoder.getPulses() - snap_eol_R;

    float avg_counts = (dL + dR) * 0.5f;
    // convert encoder counts → meters
    float travelled_m = avg_counts / COUNTS_PER_M;
    // Remaining distance to stop
    float remaining_m = (STOP_DISTANCE_MM / 1000.0f) - travelled_m;
    // if we've reached or passed stop point → stop fully
    if (remaining_m <= 0.0f) {
        target_speed_L = 0.0f;
        target_speed_R = 0.0f;
        current_state  = STOPPED;
        return;
    }
    // Normalised progress (0 → 1)
    float t = remaining_m / (STOP_DISTANCE_MM / 1000.0f);
    // clamp for safety
    if (t > 1.0f) t = 1.0f;
    if (t < 0.0f) t = 0.0f;
    // Smooth braking curve (better than linear)
    // quadratic slowdown = smoother near stop!!! note the difference 
    float speed = base_speed_ms * (t * t);

    // prevent "stall creep"    
    if (speed < 0.03f) {
        speed = 0.0f;
    }
    target_speed_L = speed;
    target_speed_R = speed;
    return;
}

/*======================== BATTERY MONITOR ========================*/

void update_battery_limit() {
    VoltageReading = ReadVoltage();
    Voltage = VoltageReading * 0.00976f;

    CurrentReading = ReadCurrent();
    Current = CurrentReading / 6400.0f;

    float voltage_scale = 1.0f;
    float current_scale = 1.0f;

    low_battery = false;
    high_current = false;

    // Voltage-based reduction
    if (Voltage <= VOLT_WARN) {
        low_battery = true;

        if (Voltage <= VOLT_MIN) {
            voltage_scale = MIN_SPEED_SCALE;
        } else {
            // linear scaling between VOLT_WARN and VOLT_MIN
            float ratio = (Voltage - VOLT_MIN) / (VOLT_WARN - VOLT_MIN);
            voltage_scale = MIN_SPEED_SCALE + ratio * (1.0f - MIN_SPEED_SCALE);
        }
    }

    // Current-based reduction
    if (Current >= CURR_WARN) {
        high_current = true;

        if (Current >= CURR_MAX) {
            current_scale = MIN_SPEED_SCALE;
        } else {
            // linear scaling between CURR_WARN and CURR_MAX
            float ratio = (CURR_MAX - Current) / (CURR_MAX - CURR_WARN);
            current_scale = MIN_SPEED_SCALE + ratio * (1.0f - MIN_SPEED_SCALE);
        }
    }

    // Use the most restrictive one
    battery_scale = (voltage_scale < current_scale) ? voltage_scale : current_scale;

    // safety clamp
    if (battery_scale < MIN_SPEED_SCALE) battery_scale = MIN_SPEED_SCALE;
    if (battery_scale > 1.0f) battery_scale = 1.0f;
}


/*======================== LINE PID LOOP ========================*/
//bool lost_search_left = true;
//bool lost_search_right = true;

void pid_control_loop() {
    read_sensors();

    update_battery_limit();

    error_val = calculate_error();

    if (emergency_stop || current_state == TURNING){
        return;
    }
    
    switch (current_state) {
        case FOLLOWING:
            // 1. Check for EOL FIRST. 
            // If we see a flat black surface, we don't care about the PID error.
            /*if (!eol_active && end_of_line_detected()) {
                snap_eol_L = left_encoder.getPulses();
                snap_eol_R = right_encoder.getPulses();
                current_state = BRAKING;
                eol_active = true;
                return; // Exit following immediately
            }*/

            // 2. PID Calculations
            integral   += error_val;
            derivative  = error_val - last_error;
            correction  = (error_val  * line_Kp) + (derivative * line_Kd);
            last_error  = error_val;

            target_speed_L = base_speed_ms + correction;
            target_speed_R = base_speed_ms - correction;

            float total_signal = 0;
            for(int i=0; i<6; i++) total_signal += sensor_values[i];
            
            if (total_signal < 0.05f) {
                if (was_centered) {
                    // was on line, now nothing → end of line
                    snap_eol_L = left_encoder.getPulses();
                    snap_eol_R = right_encoder.getPulses();
                    current_state = BRAKING;
                } else {
                    // never was centred → genuinely lost
                    current_state = LOST;
                    lost_counter  = 0;
                }
            }
            break;

        case LOST: {
            lost_counter++;

            float total_signal = 0.0f;
            float minv = 1.0f;
            float maxv = 0.0f;

            for (int i = 0; i < 6; i++) {
                float v = sensor_values[i];
                total_signal += v;
                if (v < minv) minv = v;
                if (v > maxv) maxv = v;
            }

            float variance = fabs(maxv - minv);

            bool line_found =
                (total_signal > 0.12f) &&
                (variance > 0.12f) &&
                ((sensor_values[2] > 0.25f) || (sensor_values[3] > 0.25f));

            if (line_found) {
                current_state = FOLLOWING;
                lost_counter = 0;
                break;
            }

            float search_turn = 0.10f;
            float forward = 0.08f;

            if (last_error > 0) {
                if(lost_counter < 15){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 30){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
                if(lost_counter < 50){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 75){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
                if(lost_counter < 105){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 140){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
            } else {if(lost_counter < 15){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 30){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
                if(lost_counter < 50){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 75){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
                if(lost_counter < 105){
                    target_speed_L = forward - search_turn;
                    target_speed_R = forward + search_turn;
                }
                if(lost_counter < 140){
                    target_speed_L = forward + search_turn;
                    target_speed_R = forward - search_turn;
                }
            }

            if (lost_counter > 150) current_state = STOPPED;
            break;
        }


        case STOPPED:
            target_speed_L = 0.0f;
            target_speed_R = 0.0f;
            break;

        case BRAKING:
            do_braking();
            break;

        default:
            current_state =STOPPED;
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

        if (avg >= COUNTS_180)
            break;
    }

    stop_motors();

    uL_prev = 0.5f;
    uR_prev = 0.5f;
    eL_prev = eR_prev = 0.0f;
    eL_prev2 = eR_prev2 = 0.0f;
    integral = 0.0f;
    last_error = 0.0f;
    
    was_centered = false;  // must re-centre on return run before EOL can fire
    snap_eol_L = snap_eol_R = 0;
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
        switch (current_state){
            case(FOLLOWING):{hm10.printf("FOLLOWING\r\n");break;}
            case(LOST):{hm10.printf("LOST\r\n");break;}
            case(STOPPED):{hm10.printf("STOPPED\r\n");break;}
            case(TURNING):{hm10.printf("TURNING\r\n");break;}
            case(BRAKING):{hm10.printf("BRAKING\r\n");break;}
        }
        if (hm10.readable()) {
            s = hm10.getc();
            pc.printf("BLE: %c\r\n", s);

            if (s == '1') {
                emergency_stop = false;
                was_centered   = false;  // ← ADD
                snap_eol_L = snap_eol_R = 0;  // ← ADD
                current_state  = FOLLOWING;
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
