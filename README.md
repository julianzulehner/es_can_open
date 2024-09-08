# ESP CAN

This project contains the code for a CAN gateway from J1939 to CAN open.

## Description

This repository contains code specific for the usage of CAN communication with
ESP32 development boards. <br>

Two ESP32 development boards are connected together communicating via UART.<br>

The first ESP32 reads the CAN messages from the TE connectivity OPS3 oil 
property sensor, which uses J1939 sensor at a baud rate of250kBits/s.
The data is parsed and sent via UART to the second device. <br>

The second ESP32 incooporates the CANopen application layer and works on a 
baud rate of 125kBits/s. Parts of CiA 301, CiA 305 and CiA 404 are implemented.
- NMT slave
- Node guarding
- SDO service
- TPDO service on sync
- LSS protocol (CiA 305)
- TPDO mapping according to CiA 404 (Analog input sensor)

This project was made from scratch for educational reasons. If someone is 
planning to use CANOpen with ESP32 I highly recommend the 
[CANopenNode](https://github.com/CANopenNode/CANopenNode) repository instead.
<br> 

In any case, feel free to use this code as an example or as a reference.

## Getting Started

### Dependencies
- [ESP-IDF toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) in VS Code (for straightforward compiling and flashing).


## Version History

- v1.0.2
    - First working version with incomplete implementations
    - Only one TPDO configured
    - TPDO transmit on every sync message indepent of configuration

## License

This project is licensed under the MIT License - see the LICENSE file for details

## Acknowledgments
- [CANopenNode](https://github.com/CANopenNode/CANopenNode)
