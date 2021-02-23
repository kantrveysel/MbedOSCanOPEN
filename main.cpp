#include "mbed.h"
#include <chrono>
#include <cstdio>
//#include <cstdint>
//#include <cstdio>
using duration = std::chrono::duration<int, std::milli>;

#define T_SDO_CAN_ID 0x600
#define R_SDO_CAN_ID 0x580
duration syncTime = 1s;

class PDO{
    public:
        int _currentData;
        char _NodeId,Res;
        char ControllerTemp, MotorTemp;
        int ControlWord=0x0F, TargetVelocity, TargetTorque, TargetPosition, StatusWord;
        int PositionActualValue, TorqueActualValue, DCLinkVoltage, LogicPowerSupplyVoltage;
        int CurrentDemand, MotorCurrentActualValue, ElectricalAngle, PhaseACurrent, PhaseBCurrent;

    PDO(char NodeId){
        _NodeId = NodeId;
        ControlWord = 0x0F;
        Res = 0x0;
    }

    bool isPDO(CANMessage msg){
        if ((msg.id == 0x180 + _NodeId) || (msg.id == 0x280 + _NodeId) || (msg.id == 0x380 + _NodeId) || (msg.id == 0x480 + _NodeId)){
            return true;
        }
            return false;
    }

    void readPDO(CANMessage msg){
        if (msg.id == 0x180 + _NodeId){ // TPDO1
            StatusWord = msg.data[0]*255 + msg.data[1];
            PositionActualValue = msg.data[2]*255^3 + msg.data[3]*255^2 + msg.data[4]*255 + msg.data[5];
            TorqueActualValue = msg.data[6]*255 + msg.data[7];
        
        }else if (msg.id == 0x280 + _NodeId){ // TPDO2
            ControllerTemp = msg.data[0];
            MotorTemp = msg.data[1];
            DCLinkVoltage = msg.data[2]*255 + msg.data[3];
            LogicPowerSupplyVoltage = msg.data[4]*255 + msg.data[5];
            CurrentDemand = msg.data[6]*255 + msg.data[7];

        }else if (msg.id == 0x380 + _NodeId){ // TPDO3
            MotorCurrentActualValue = msg.data[0]*255 + msg.data[1];
            ElectricalAngle = msg.data[2]*255 + msg.data[3];
            PhaseACurrent = msg.data[4]*255 + msg.data[5];
            PhaseBCurrent = msg.data[6]*255 + msg.data[7];
            
        }else if (msg.id == 0x480 + _NodeId){ // TPDO4

        }
    }
    CANMessage sendPDO(char pdo, char data[]){
        char _lenData = sizeof(*data);
        int _id = 256 + pdo*256 + _NodeId;
        return CANMessage(_id, data, _lenData);
    }
    
    void writePDO(char pdo, char data[]);

    void updatePDO(){
        char data[8];
        data[0] = ControlWord;
        data[1] = 0;
        data[2] = TargetVelocity%256;
        data[3] = (TargetVelocity/256)%256;
        data[4] = ((TargetVelocity/256)/256)%256;
        data[5] = (((TargetVelocity/256)/256)/256)%256;
        data[6] = TargetTorque%256;
        data[7] = (TargetTorque/256)%256;
        writePDO(1,data);

        data[0] = TargetPosition%256;
        data[1] = (TargetPosition/256)%256;
        data[2] = ((TargetPosition/256)/256)%256;
        data[3] = (((TargetPosition/256)/256)/256)%256;
        data[4] = Res;
        data[5] = Res;
        data[6] = Res;
        data[7] = Res;
        writePDO(2,data);
    }
};

char counter = 0;
char motorNodeID = 10;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

CAN can1(PD_0, PD_1, 500000);

//CAN can1(PB_12, PB_13);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;

PDO myPDO(1);

void oku();

void PDO::writePDO(char pdo, char data[]){
    char _lenData = sizeof(*data);
    int _id = 256 + pdo*256 + motorNodeID;
    can1.write(CANMessage(_id, data, _lenData));
}

void syncCanOpen(){
    myPDO.writePDO(1, 0);
    char data = 0x00;
    can1.write(CANMessage(0x080, &data, 1));
    led2 = !led2;
}

int * signed16(int x){

    static int ret[4];
    for(int i=0 ; i<4 ; i++){
        ret[i] = x%256;
        x /=256;
    }
    return ret;

}

int resigned16(char x[]){
    static int ret = 0;
    for(int i=0; i<4; i++){
        ret += x[i]*256^i;
    }
    return ret;
}

void setSDO(int CAN_ID, int main_index, int sub_index, int data){

    printf("SDO Setting %d / %d from %X to %d \n",main_index, sub_index, CAN_ID, data);
    static char msg[8];
    msg[0] = 0x2B; // Command Byte

    static int *_mainindex = signed16(main_index);
    msg[1] = *(_mainindex);
    msg[2] = *(_mainindex+1); // Main - index

    msg[3] = sub_index; // Sub - index

    static int *datas = signed16(data);
    for(int i=4; i<8; i++){
        msg[i] = *(datas+ i-4 );
    }
    
    can1.write( CANMessage( CAN_ID, msg, sizeof(msg) ));
    led2 = !led2;
    }

void requestSDO(int CAN_ID, int main_index, int sub_index){

    printf("SDO Requested %d / %d from %X \n",main_index, sub_index, CAN_ID);
    static char msg[8];
    msg[0] = 0x40;

    static int *_mainindex = signed16(main_index);
    msg[1] = *(_mainindex);
    msg[2] = *(_mainindex+1);

    msg[3] = sub_index;
     for(int i=4; i<8; i++){
        msg[i] = 0x00;
    }

    can1.write( CANMessage( CAN_ID, msg, sizeof(msg) ));
    led2 = !led2;
}

void CanOpenInit(){

    char data[2];
    bool status[2];

    printf("Pre-Operational Mod Initializing...\n");
    // Pre-Operational Mode
    data[0] = 0x80;
    data[1] = 0x00;
    status[0] = can1.write(CANMessage(0x0, data, sizeof(data)));
    data[0] = 0x80;
    data[1] = motorNodeID;
    status[1] = can1.write(CANMessage(0x0, data, sizeof(data)));
    led1 = !led1;
    if(status[0] && status[1])
        printf("Pre-Operational Mod Initilalized\n");
    oku();

    printf("Operational Mod Initializing...\n");
    // Operational Mode
    data[0] = 0x01;
    data[1] = 0x00;
    status[0] = can1.write(CANMessage(0x0, data, sizeof(data)));
    data[0] = 0x01;
    data[1] = motorNodeID;
    status[1] = can1.write(CANMessage(0x0, data, sizeof(data)));
    led1 = !led1;
    if(status[0] && status[1])
        printf("Operational Mod Initilalized\n");
    oku();

    requestSDO(T_SDO_CAN_ID + motorNodeID, 0x1810, 0x01); // Check vendor ID
    oku();

    char datas[8] = {0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Setting Control word to 6, 7, 15
    can1.write(CANMessage(0x200 + motorNodeID, datas, sizeof(datas)));
    oku();
    datas[0] = 0x07;
    can1.write(CANMessage(0x200 + motorNodeID, datas, sizeof(datas)));
    oku();
    datas[0] = 0x0F;
    can1.write(CANMessage(0x200 + motorNodeID, datas, sizeof(datas)));
    oku();

}

void oku(){

    CANMessage msg;
    if(can1.read(msg)) {


        if (myPDO.isPDO(msg)) {
            printf("PDO ");
            myPDO.readPDO(msg);
        }
        
        led3 = !led3;

        if(msg.id == R_SDO_CAN_ID + motorNodeID){ // SDO Respond
            printf("SDO ");
            if(msg.data[0] == 0x60){ // SDO download (setting)
                printf("Setting Successful %Xh / %Xh\n",msg.data[1] + msg.data[2]*256, msg.data[3]);
            }

            if(msg.data[0] == 0x4B){ // SDO Upload (reading)
                printf("Upload Successful %Xh / %Xh\n",msg.data[1] + msg.data[2]*256, msg.data[3]);            
                char data[4];
                printf("Recieved Data : ");
                for(int i=4; i<msg.len; i++){
                    data[i-4] = msg.data[i];
                    printf("[%Xh] ",msg.data[i]);
                }
                printf("\n");
                printf("\t Recieved Data : %d\n",resigned16(data));
            }
        }
        
        if(true){ // SHOW MESSAGE CONTENT
            printf("Msg ID: %Xh / ",msg.id);
            
            printf("Content :");
            for(int i=0; i<msg.len; i++){
                printf(" [%Xh]",msg.data[i]);
            }
            printf("\n");
        }
    }
}

void WaitForEMCU(){
    CANMessage msg;
    while(1) {
        if(can1.read(msg)) {
            if (msg.data[0] == 0 && msg.len == 1){            
                motorNodeID = msg.id - 0x700;
                myPDO._NodeId = motorNodeID;

                printf("MCU Connected by Node : %d\n", motorNodeID);
                led1 = 0;
                break;
            }

            printf("Msg ID: %Xh , Node ID : %X \n",msg.id, msg.id - 0x700);
               
        } 
        thread_sleep_for(100);
        led1 = !led1;
    }
}

int main() {
    printf("START\n");
    can1.frequency(500000); // 500 kb/s

   
    t.start(callback(&queue, &EventQueue::dispatch_forever)); // heart-beat
    queue.call_every(syncTime, syncCanOpen); 
    

    CANMessage msg;

    // Waiting For Boot-Up
    printf("Waiting For EMCU\n");
    WaitForEMCU();

    CanOpenInit();
    
    requestSDO(T_SDO_CAN_ID + motorNodeID, 0x2033, 0x00); // Data Request Test
    
    while(true){

        printf("Motor Temp\t : \t%d\n",           myPDO.MotorTemp);
        printf("Torque Actual Value\t : \t%d\n",  myPDO.TorqueActualValue);
        printf("Controller Temp\t : \t%d\n",      myPDO.ControllerTemp);
        
        oku();

    }
}
