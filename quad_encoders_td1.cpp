#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

constexpr int PPR = 256;

QEI left_encoder(PA_6, PA_7, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_6, PB_7, NC, PPR, QEI::X4_ENCODING);


Ticker speedTicker;
DigitalOut led(D5); //red led 

//initialise
volatile int last_tick_left = 0, last_tick_right = 0; //previous encoder counts 

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
    C12832 lcd(D11, D13, D12, D7, D10); 
    lcd.cls();
    lcd.locate(20,0);
    speedTicker.attach(&speed_tick, 50ms); //20 Hz speed update

    while(true){
        lcd.printf("Left: %d, Right: %d\n", left_encoder.getPulses(), right_encoder.getPulses());
        ThisThread::sleep_for(500ms);
    }
}