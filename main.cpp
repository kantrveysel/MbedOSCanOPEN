// EXAMPLE

#include "mbed.h"
#include "mbed-os\CANOpen.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;
CAN can1(PD_0, PD_1, 500000);
CANOpen CO(can1, 1);

int Torq;

int main(){

    t.start(callback(&queue, &EventQueue::dispatch_forever)); // heart-beat
    queue.call_every(CO.syncTime, CO.syncCanOpen); 
    
    CANMessage msg;
    while(true){
        if (can1.read(msg)) {

            CO.syncCanOpen();


            CO.readPDO(msg, 1, 0, 1, Torq);
            CO.requestSDO(0x6040, 0x40);
            led1 != led1;
            if(CO.readError(msg) != 0x0){
            
                printf("! ERROR ! %#X \n", CO.readError(msg));
                led3 != led3;
                
            }
    }
    }
}
