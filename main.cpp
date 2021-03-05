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
int voltage;

int main(){

    t.start(callback(&queue, &EventQueue::dispatch_forever)); // Setup for SYNC
    queue.call_every(CO.syncTime, CO.syncCanOpen); // Starts SYNC , it is necessary for reading PDO
    
    CANMessage msg; // Create CANMessage for reading
    while(true){
        
        readSDO(0x6040, 0x0, voltage); // main index 0x6040 , sub index 0x0 , save to voltage
        
        if (can1.read(msg)) { // is there any data

            CO.readPDO(msg, 1, 0, 2, Torq); // Actual Torq is readable from RPDO1 index 0 and 2 byte length
            led1 != led1; // Show me there is a data
            if(CO.readError(msg) != 0x0){ // if there is no error then return 0x0 or return error
            
                printf("! ERROR ! %#X \n", CO.readError(msg));
                led3 != led3; // RED Light
                
            }
        }else{
            
                led2 != led2; // No data on CAN
                
         }
        
        printf(" Voltage : %d\n Torque : %d\n", voltage, Torq); // All data we collect
    }
}
