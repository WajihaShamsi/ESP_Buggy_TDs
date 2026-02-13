#include "mbed.h"
#include "QEI.h"

constexpr int PPR = 256;

QEI left_encoder(PA_6, PA_7, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_6, PB_7, NC, PPR, QEI::X4_ENCODING);


Ticker speedTicker;
DigitalOut led(D5); //red led 

//initialise
volatile int last_tick_left = 0, last_tick_right = 0; //previous encoder counts 
volatile float cps_left = 0.0f, cps_right = 0.0f;   // counts per second

void speed_tick(){
    const int now_left = left_encoder.getPulses();    //new encoder counts
    const int now_right = right_encoder.getPulses();   

    const int dL = now_left - last_tick_left;      //difference
    const int dR = now_right - last_tick_right;

    last_tick_left = now_left; //update
    last_tick_right = now_right;

    if(dL !=0 || dR !=0){
        led = !led; // toggle an LED here
    }
}

int main(){ 
    speedTicker.attach(&speed_tick, 50ms); //20 Hz speed update
    while (true) {
        // here write the logic for rpm, speed etc etc ill do later

    }
}