//
// Created by czy04 on 2024/5/10.
//

#include "modbus.h"

void modbus_backend::Handle() {
    uint8_t exception = MB_EX_OK;
    if (rx_msg.len > 0 && rx_msg.data[MB_DATA_OFF] == MB_EX_OK) {
        switch (rx_msg.data[MB_FUNC_OFF]) {
            case MB_FC_READ_COILS:
                if (addr > nb_bits) {
                    exception = MB_EX_ILLEGAL_DATA_ADDRESS;
                } else if (addr + nb > nb_bits) {
                    exception = MB_EX_ILLEGAL_DATA_VALUE;
                } else
                    tx_len = read_coils(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], backend_data) +
                             1; // 1 = function code
                break;
            case MB_FC_READ_DISCRETE_INPUTS:
                if (addr > nb_input_bits) {
                    exception = MB_EX_ILLEGAL_DATA_ADDRESS;
                } else if (addr + nb > nb_input_bits) {
                    exception = MB_EX_ILLEGAL_DATA_VALUE;
                } else
                    tx_len = read_discrete_inputs(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3],
                                                  backend_data) + 1;
                break;
            case MB_FC_READ_HOLDING_REGISTERS:
                if (addr > nb_regs) {
                    exception = MB_EX_ILLEGAL_DATA_ADDRESS;
                } else if (addr + nb > nb_regs) {
                    exception = MB_EX_ILLEGAL_DATA_VALUE;
                } else
                    tx_len = read_holding_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3],
                                                    (uint16_t *) backend_data) + 1;
                break;
            case MB_FC_READ_INPUT_REGISTERS:
                if (addr > nb_input_regs) {
                    exception = MB_EX_ILLEGAL_DATA_ADDRESS;
                } else if (addr + nb > nb_input_regs) {
                    exception = MB_EX_ILLEGAL_DATA_VALUE;
                } else
                    tx_len = read_input_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3],
                                                  (uint16_t *) backend_data) + 1;
                break;
            case MB_FC_WRITE_SINGLE_COIL:
                write_single_coil(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3]);
                break;
            case MB_FC_WRITE_SINGLE_REGISTER:
                write_single_register(rx_msg.data[MB_DATA_OFF + 1],
                                      (rx_msg.data[MB_DATA_OFF + 3] << 8) + rx_msg.data[MB_DATA_OFF + 4]);
                break;
            case MB_FC_WRITE_MULTIPLE_COILS:
                write_multiple_coils(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3], backend_data);
                break;
            case MB_FC_WRITE_MULTIPLE_REGISTERS:
                write_multiple_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3],
                                         (uint16_t *) backend_data);
                break;
            case MB_FC_READWRITE_MULTIPLE_REGISTERS:
                exception = read_write_multiple_registers(rx_msg.data[MB_DATA_OFF + 1], rx_msg.data[MB_DATA_OFF + 3],
                                                          (uint16_t *) backend_data, rx_msg.data[MB_DATA_OFF + 5],
                                                          rx_msg.data[MB_DATA_OFF + 7],
                                                          (uint16_t * )(backend_data + 2));
                break;
            default:
                break;
        }
    } else return;
    if (exception != MB_EX_OK) {
        tx_msg.data[MB_FUNC_OFF] = rx_msg.data[MB_FUNC_OFF] + 0x80;
        tx_msg.data[MB_DATA_OFF] = exception;
        tx_msg.len = 2;
    } else {
        tx_msg.len = tx_len;
        tx_msg.data[MB_FUNC_OFF] = rx_msg.data[MB_FUNC_OFF]; // function code
        memcpy(tx_msg.data + 1, tx_data, tx_len - 1); // copy obtained data to tx_msg
    }
}

uint8_t modbus_backend::read_coils(uint16_t addr, uint16_t nb, uint8_t *dest) { // TODO: 读取时的错误处理及返回值
    dest = new uint8_t[nb / 8 + 1];
    memset(dest, 0, nb / 8 + 1);
    for (int i = 0; i < nb / 8; i++) {
        memcpy(dest + i, bits_start + addr + i, 1);
    }
    for (int i = 0; i < nb % 8; ++i) {
        dest[nb / 8] |= bits_start[addr + nb / 8] & (1 << i);
    }
    return nb / 8 + 1;
}

uint8_t modbus_backend::read_discrete_inputs(uint16_t addr, uint16_t nb, uint8_t *dest) {

    dest = new uint8_t[nb / 8 + 1];
    memset(dest, 0, nb / 8 + 1);
    for (int i = 0; i < nb / 8; i++) {
        memcpy(dest + i, input_bits_start + addr + i, 1);
    }
    for (int i = 0; i < nb % 8; ++i) {
        dest[nb / 8] |= input_bits_start[addr + nb / 8] & (1 << i);
    }
    return nb / 8 + 1;
}

uint8_t modbus_backend::read_holding_registers(uint16_t addr, uint16_t nb, uint16_t *dest) {
    for (int i = 0; i < nb; i++) {
        dest[i] = (regs_start[addr + i * 2] << 8) + regs_start[addr + i * 2 + 1];
    }
    return nb;
}

uint8_t modbus_backend::read_input_registers(uint16_t addr, uint16_t nb, uint16_t *dest) {
    for (int i = 0; i < nb; i++) {
        dest[i] = (input_regs_start[addr + i * 2] << 8) + input_regs_start[addr + i * 2 + 1];
    }
    return nb;
}


void modbus::Handle() {
    if (backend->tx_msg.len > 0) {
        prepare_response(); // 首先准备上一条消息的回复
    }
    if (backend->rx_msg.len > 0) {
        uint8_t exception_code = pre_check(); // 预检查
        if (exception_code == MB_EX_OK) {
            save_to_backend(); // 保存到backend
        } else return;
    }
}

void modbus::pre_check() {

// address check
    if (backend->rx_msg.data[0] != address) {
        return
                MB_EX_NONE;
    }
// CRC check
    uint16_t crc = (backend->rx_msg.data[backend->rx_msg.len - 2] << 8) +
                   backend->rx_msg.data[backend->rx_msg.len - 1]; // 最后两个字节为CRC
    if (crc !=
        CRC16(backend
                      ->rx_msg.data, backend->rx_msg.len - 2)) {
        return
                MB_EX_NONE;
    }
    return
            MB_EX_OK;
}

// CRC16
uint16_t modbus::CRC16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus::save_to_backend() {
    backend->rx_len = rx_msg.len;
    backend->function_code = rx_msg.data[MB_FUNC_OFF]; // 将function code和data分别保存
    memcpy(backend->data, &(rx_msg.data[MB_DATA_OFF + MB_PDU_OFF]),
           rx_msg.len - 4); // 4 = address(1) + function code(1) + CRC(2)
}

uint8_t modbus::prepare_response() {
    tx_msg.data[0] = address;
    tx_msg.data[1] = backend->function_code; // 此时backend中储存的function code应该是上一条消息的function code
    memcpy(&(tx_msg.data[2]), backend->data, backend->rx_len - 4);
    uint16_t crc = CRC16(tx_msg.data, backend->rx_len - 2);
    tx_msg.data[backend->rx_len - 2] = crc >> 8;
    tx_msg.data[backend->rx_len - 1] = crc & 0xFF;
    tx_msg.len = backend->rx_len;
    return MB_EX_OK;
}
