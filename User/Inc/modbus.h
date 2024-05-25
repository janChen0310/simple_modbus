//
// Created by czy04 on 2024/5/10.
//

//modbus协议层的数据结构和函数声明

#ifndef MODBUS_MODBUS_H
#define MODBUS_MODBUS_H

#include "main.h"
#include "modbus_protocol.h"

typedef struct {
    uint8_t data[MB_ADU_SIZE_MAX];
    uint8_t len;
} Message;

class modbus;

class modbus_backend { // modbus 后端
private:
    uint8_t* bits_start; // 线圈起始地址
    uint8_t nb_bits; // 线圈数量
    uint8_t* input_bits_start; // 输入线圈起始地址
    uint8_t nb_input_bits; // 输入线圈数量
    uint8_t* regs_start; // 寄存器起始地址
    uint8_t nb_regs; // 寄存器数量
    uint8_t* input_regs_start; // 输入寄存器起始地址
    uint8_t nb_input_regs; // 输入寄存器数量

    uint8_t function_code;
    uint8_t rx_data[MB_PDU_SIZE_MAX - 1]; // modbus类将接收到的消息预处理后存入rx_data
    uint8_t* tx_data; // tx_data指向一块动态分配的内存，内存大小根据读取的数据大小确定
    uint8_t rx_len, tx_len; // rx_len从modbus处获取，tx_len由计算得到
public:
    inline modbus_backend(uint8_t* _bits_start,uint8_t _nb_bits,
                   uint8_t* _input_bits_start, uint8_t _nb_input_bits,
                   uint8_t* _regs_start, uint8_t _nb_regs,
                   uint8_t* _input_regs_start, uint8_t _nb_input_regs)
    : bits_start(_bits_start), nb_bits(_nb_bits),
        input_bits_start(_input_bits_start), nb_input_bits(_nb_input_bits),
        regs_start(_regs_start), nb_regs(_nb_regs),
        input_regs_start(_input_regs_start), nb_input_regs(_nb_input_regs) {
        }


    void Handle();

    // 主机模式下为设置通信从机地址，从机模式下为设置自身地址
    uint8_t set_address(uint8_t address);
    uint8_t get_address();
    uint8_t set_timeout(uint32_t timeout);
    uint8_t get_timeout();

    uint8_t read_coils(uint16_t addr, uint16_t nb, uint8_t *dest);
    uint8_t read_discrete_inputs(uint16_t addr, uint16_t nb, uint8_t *dest);
    uint8_t read_holding_registers(uint16_t addr, uint16_t nb, uint16_t *dest);
    uint8_t read_input_registers(uint16_t addr, uint16_t nb, uint16_t *dest);
    uint8_t write_single_coil(uint16_t addr, uint8_t value);
    uint8_t write_single_register(uint16_t addr, uint16_t value);
    uint8_t write_multiple_coils(uint16_t addr, uint16_t nb, uint8_t *src);
    uint8_t write_multiple_registers(uint16_t addr, uint16_t nb, uint16_t *src);
    uint8_t read_write_multiple_registers(uint16_t read_addr, uint16_t read_nb, uint16_t *dest, uint16_t write_addr, uint16_t write_nb, uint16_t *src);

};

class modbus { // modbus协议解析
private:

public:
    void Handle();

    Message rx_msg, tx_msg; // Modbus message 接收的消息及长度从modbus_frontend的串口处获取，发送的消息及长度从modbus_backend处获取
    uint8_t address; // Modbus address
    modbus_backend *backend;
    uint8_t pre_check();
    uint8_t save_to_backend();
    uint8_t prepare_response();
    uint16_t CRC16(uint8_t *data, uint16_t len);
};

#endif //MODBUS_MODBUS_H