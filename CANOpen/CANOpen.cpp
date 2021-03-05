#include "mbed.h"
#include "CANOpen.h"

CANOpen::CANOpen(CAN &_can, char NodeId) : can(_can){

    _NodeId = NodeId;

}


int * CANOpen::signed16(int x){
    static int ret[4];
    for(int i=0 ; i<4 ; i++){
        ret[i] = x%256;
        x /=256;
    }
    return ret;
}

void CANOpen::requestSDO(int main_index, int sub_index){
    static char msg[8];
    msg[0] = 0x40;

    static int *_mainindex = signed16(main_index);
    msg[1] = *(_mainindex);
    msg[2] = *(_mainindex+1);

    msg[3] = sub_index;
    for(int i=4; i<8; i++){
        msg[i] = 0x00;
    }

    can.write( CANMessage( T_SDO_CAN_ID + _NodeId, msg, sizeof(msg) ));
}

void CANOpen::setSDO(int main_index, int sub_index, int data){

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
    
    can.write( CANMessage( R_SDO_CAN_ID + _NodeId, msg, sizeof(msg) ));
}

void CANOpen::syncCanOpen(){
    char data = 0x00;
    can.write(CANMessage(0x080, &data, 1));
}

void CANOpen::updatePDO(char pdo){// Depends on your CANOpen device this is for e-drive

        char data[8];

        if(pdo == 1){ 
            data[0] = ControlWord;
            data[1] = 0;
            data[2] = TargetVelocity%256;
            data[3] = (TargetVelocity/256)%256;
            data[4] = ((TargetVelocity/256)/256)%256;
            data[5] = (((TargetVelocity/256)/256)/256)%256;
            data[6] = TargetTorque%256;
            data[7] = (TargetTorque/256)%256;
            writePDO(pdo,data);
        }else if(pdo == 2){
            data[0] = TargetPosition%256;
            data[1] = (TargetPosition/256)%256;
            data[2] = ((TargetPosition/256)/256)%256;
            data[3] = (((TargetPosition/256)/256)/256)%256;
            data[4] = Res;
            data[5] = Res;
            data[6] = Res;
            data[7] = Res;
            writePDO(pdo,data);
        }
}

void CANOpen::writePDO(char pdo, char data[]){

    char _lenData = sizeof(*data);
    int _id = 256 + pdo*256 + _NodeId;
    can.write(CANMessage(_id, data, _lenData));

}

void CANOpen::readPDO(CANMessage msg, char pdo, char startByte, char size, int &a){
    int ret = 0;
    if(msg.id == 256 + pdo*256 +_NodeId){
        char n = 0;
        for(int i = startByte; i <= size; i++){
            ret += msg.data[i] * (255^n);
            n++;
        }
        a = ret;
    }
}

int  CANOpen::readError(CANMessage msg){
    if(msg.id == 0x80 + _NodeId){
        return msg.data[0] + msg.data[1]*256;
    }
    return 0x0;
}

void CANOpen::readSDO(int main_index, char sub_index, int &a){

    requestSDO(main_index, sub_index);
    CANMessage msg;
    can.read(msg);
    if(msg.id == R_SDO_CAN_ID + _NodeId){
        if(msg.data[1] + msg.data[2]*256 == main_index & msg.data[3] == sub_index){

            a = msg.data[4] + msg.data[5]*256 + msg.data[6]*256^2 + msg.data[4]*256^3;

        }
    }
}
