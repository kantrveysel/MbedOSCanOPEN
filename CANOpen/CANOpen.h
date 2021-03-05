#include "mbed.h"

#define T_SDO_CAN_ID            0x600
#define R_SDO_CAN_ID            0x580

using duration = std::chrono::duration<int, std::milli>;

class CANOpen{

    private:
        CAN &can;
        char motorNodeID = 10;
        int _currentData;
        char _NodeId,Res;

    public:
        CANOpen(CAN &_can, char _NodeId);
        
        duration syncTime = 1s;
        char ControllerTemp, MotorTemp;
        int ControlWord=0x0F, TargetVelocity, TargetTorque, TargetPosition, StatusWord;
        int PositionActualValue, TorqueActualValue, DCLinkVoltage, LogicPowerSupplyVoltage;
        int CurrentDemand, MotorCurrentActualValue, ElectricalAngle, PhaseACurrent, PhaseBCurrent;
        
        void requestSDO(int main_index, int sub_index);
        void setSDO(int main_index, int sub_index, int data);
        int * signed16(int x);
        void syncCanOpen();
        void updatePDO(char pdo);
        void writePDO(char pdo, char data[]);
        void readPDO(CANMessage msg, char pdo, char startByte, char size, int &a);

};
