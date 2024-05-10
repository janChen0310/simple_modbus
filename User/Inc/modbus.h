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
public:
    modbus_backend();

    void Handle()
    {
        if (rx_msg.len > 0 && rx_msg.data[MB_DATA_OFF] == MB_EX_OK) {
            switch (rx_msg.data[MB_FUNC_OFF])
            {
                case MB_FC_READ_COILS:
                    read_coils(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], backend_data);
                    break;
                case MB_FC_READ_DISCRETE_INPUTS:
                    read_discrete_inputs(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], backend_data);
                    break;
                case MB_FC_READ_HOLDING_REGISTERS:
                    read_holding_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], (uint16_t *)backend_data);
                    break;
                case MB_FC_READ_INPUT_REGISTERS:
                    read_input_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], (uint16_t *)backend_data);
                    break;
                case MB_FC_WRITE_SINGLE_COIL:
                    write_single_coil(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3]);
                    break;
                case MB_FC_WRITE_SINGLE_REGISTER:
                    write_single_register(rx_msg.data[MB_DATA_OFF + 1], (rx_msg.data[MB_DATA_OFF + 3] << 8) + rx_msg.data[MB_DATA_OFF + 4]);
                    break;
                case MB_FC_WRITE_MULTIPLE_COILS:
                    write_multiple_coils(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], backend_data);
                    break;
                case MB_FC_WRITE_MULTIPLE_REGISTERS:
                    write_multiple_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], (uint16_t *)backend_data);
                    break;
                case MB_FC_READWRITE_MULTIPLE_REGISTERS:
                    read_write_multiple_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], (uint16_t *)backend_data, rx_msg.data[MB_DATA_OFF + 5], rx_msg.data[MB_DATA_OFF + 7], (uint16_t *)(backend_data + 2));
                    break;
                default:
                    break;
            }
        }
    }

    // 主机模式下为设置通信从机地址，从机模式下为设置自身地址
    uint8_t set_address(modbus* ctx, uint8_t address);
    uint8_t set_timeout(uint32_t timeout);
    uint8_t get_timeout();
    uint8_t address; // Modbus address
    uint8_t backend_data[MB_PDU_SIZE_MAX];
    Message rx_msg, tx_msg; // Modbus message

    uint8_t read_coils(uint16_t addr, uint16_t nb, uint8_t *dest);
    uint8_t read_discrete_inputs(uint16_t addr, uint16_t nb, uint8_t *dest);
    uint8_t read_holding_registers(uint16_t addr, uint16_t nb, uint16_t *dest);
    uint8_t read_input_registers(uint16_t addr, uint16_t nb, uint16_t *dest);
    uint8_t write_single_coil(uint16_t addr, uint8_t value);
    uint8_t write_single_register(uint16_t addr, uint16_t value);
    uint8_t write_multiple_coils(uint16_t addr, uint16_t nb, uint8_t *src);
    uint8_t write_multiple_registers(uint16_t addr, uint16_t nb, uint16_t *src);
    uint8_t read_write_multiple_registers(uint16_t read_addr, uint16_t read_nb, uint16_t *dest, uint16_t write_addr, uint16_t write_nb, uint16_t *src);

    void modbus_backend_Handle();
};

class modbus { // modbus协议解析
private:

public:
    void Handle();

    modbus_backend *backend;
    uint8_t pre_check();
    uint8_t save_to_backend();
    uint8_t save_to_backend(uint8_t exception_code);
    uint8_t prepare_response();
};

#endif //MODBUS_MODBUS_H

