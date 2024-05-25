//
// Created by czy04 on 2024/5/10.
//

//modbus通信层的数据结构和函数声明

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus.h"

extern modbus _modbus;

class modbus_frontend{
private:
    char* port; // "/dev/ttyUSB0" etc.
    int baud; // 9600, 115200 etc.
    uint8_t databits;
    uint8_t stopbits;
    uint8_t parity;
    modbus *ctx;
    uint8_t rx_buff[MB_ADU_SIZE_MAX]; // 从串口读到的数据最先被DMA搬运到这里
    uint8_t tx_buff[MB_ADU_SIZE_MAX];
public:
    inline modbus_frontend(char* port, int baud, uint8_t databits, uint8_t stopbits, uint8_t parity)
    : port(port), baud(baud), databits(databits), stopbits(stopbits), parity(parity) {
        ctx = &_modbus;
    }
    uint8_t connect(); // Connect to the serial port
    uint8_t disconnect(); // Disconnect from the serial port
    ssize_t send(uint8_t *pdu, uint8_t len); // Send a Modbus PDU
    ssize_t recv(); // Receive a Modbus PDU

    void Handle();
};

#endif //MODBUS_RTU_H
