# MbedOSCanOPEN

A lightweight **CANopen** library for **STM32 Nucleo F429ZI**, designed for electric vehicle (EV) control unit communication using **Mbed OS 6.3.0**.  
Developed for **YTU Racing** to interface with **Emsiso eDrive** systems via the CANopen protocol.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)[![GitHub Issues](https://img.shields.io/github/issues/kantrveysel/MbedOSCanOPEN)](https://github.com/kantrveysel/MbedOSCanOPEN/issues)
[![GitHub Stars](https://img.shields.io/github/stars/kantrveysel/MbedOSCanOPEN?style=flat-square)](https://github.com/kantrveysel/MbedOSCanOPEN/stargazers)

---

## 📌 Overview

This open-source CANopen library enables communication with eDrive systems in electric vehicles, supporting:

- **SDO (Service Data Object)** – Read/write configuration data (e.g., motor parameters)
- **PDO (Process Data Object)** – Real-time data exchange (e.g., torque, velocity)
- **SYNC** – Synchronized PDO updates
- **Error Handling** – Reads and reports CANopen errors

Optimized for **STM32 Nucleo F429ZI** and **Mbed OS 6.3.0**, following **Emsiso eDrive** documentation.

---

## ✨ Features

- ✅ **CANopen Protocol**: SDO (read/write), PDO (real-time), SYNC
- ✅ **STM32 Support**: Fully compatible with Nucleo F429ZI
- ✅ **eDrive Integration**: Torque, velocity, position configuration
- ✅ **Error Management**: Detects & reports motor/controller errors
- ✅ **Example Code**: Includes a working demo (`main.cpp`) for torque & voltage monitoring

---

## 🚀 Getting Started

### ✅ Prerequisites

- **Hardware**: STM32 Nucleo F429ZI  
- **Software**: Mbed OS 6.3.0, Mbed CLI  
- **CAN Interface**: Connected to eDrive system (e.g., Emsiso)  
- **Dependencies**: Mbed OS (`mbed.h`)

---

### ⚙️ Installation

```bash
git clone https://github.com/kantrveysel/MbedOSCanOPEN.git
mbed import MbedOSCanOPEN
cd MbedOSCanOPEN
```
### 🔧 Compile & Flash
```bash
mbed compile --target NUCLEO_F429ZI --toolchain GCC_ARM
mbed flash
```

### 🧠 Usage
This library provides a CANOpen class. Example usage:
```cpp
#include "mbed.h"
#include "CANOpen.h"

CAN can1(PD_0, PD_1, 500000);      // CAN at 500 kbps
CANOpen CO(can1, 1);               // Node ID 1

int main() {
    EventQueue queue(32 * EVENTS_EVENT_SIZE);
    Thread t;
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(CO.syncTime, CO.syncCanOpen);  // SYNC every 1s

    CANMessage msg;
    int torque, voltage;

    while (true) {
        CO.readSDO(0x6040, 0x0, voltage); // Read control word

        if (can1.read(msg)) {
            CO.readPDO(msg, 1, 0, 2, torque); // Read torque (RPDO1, bytes 0-2)
            if (CO.readError(msg) != 0x0) {
                printf("! ERROR ! %#X\n", CO.readError(msg));
            }
        }

        printf("Voltage: %d\nTorque: %d\n", voltage, torque);
    }
}
```


### 🔑 Key Functions
- `requestSDO` – Requests SDO data  
- `setSDO` – Writes SDO values  
- `updatePDO` – Sends PDOs for control  
- `readPDO` – Reads real-time values  
- `readError` – Returns CANopen error codes  

## ⚙️ Technical Details

- **Platform**: STM32 Nucleo F429ZI  
- **Mbed OS Version**: 6.3.0  
- **Protocol**: CANopen (SDO: `0x600/0x580`, SYNC: `0x080`)  
- **CAN Bus**: 500 kbps, configurable pins (e.g., `PD_0`, `PD_1`)


## 🔮 Future Improvements
- Replace `signed16` with `std::vector` for thread safety  
- Improve CAN read error handling  
- Optimize with bit-shifting for faster parsing  
- Add more PDO mapping options  
