#include "mbed.h"


UnbufferedSerial hm10(PA_11, PA_12);   // PA_11=TX, PA_12=RX
UnbufferedSerial pc(USBTX, USBRX);
DigitalOut LED(D5);

int main() {
    pc.baud(9600);
    hm10.baud(9600);

    char s, w;

    while (1) {
  
        if (hm10.readable()) {
            hm10.read(&s, 1); // Read 1 byte into address of 's'
            pc.write(&s, 1);  // Forward it to the PC
            
            if (s == '1'){
                LED = 1;
            }
            else if (s == '0') {
               LED = 0;
            }
        }

        // PC to HM-10 (Configuration Mode)
        if (pc.readable()) {
            pc.read(&w, 1);
            hm10.write(&w, 1);
        }
    }
}
