#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

constexpr int PPR = 256;

QEI left_encoder(PB_2, PB_1, NC, PPR, QEI::X4_ENCODING);
QEI right_encoder(PB_15, PB_14, NC, PPR, QEI::X4_ENCODING);


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
    speedTicker.attach(&speed_tick, 50ms); //20 Hz speed update

    while(true){
        int left = left_encoder.getPulses(); //added logic to displaym ticks to lcd screen
        int right = right_encoder.getPulses();

        lcd.cls();
        lcd.locate(0, 0);
        lcd.printf("Left Encoder: %d", left);

        lcd.locate(0, 20);
        lcd.printf("Right Encoder: %d", right);

        ThisThread::sleep_for(200ms);

    }
}
