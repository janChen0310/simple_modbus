//
// Created by czy04 on 2024/5/10.
//

#include "modbus.h"

void modbus_backend::Handle()
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

void modbus::Handle()
{
    if (backend->tx_msg.len > 0) {
        prepare_response();
    }
    if (backend->rx_msg.len > 0) {
        uint8_t exception_code = pre_check();
        if (exception_code == MB_EX_OK) {
            save_to_backend();
        }
        else {
            save_to_backend(exception_code);
        }
    }
}
