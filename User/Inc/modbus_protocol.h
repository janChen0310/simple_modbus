//
// Created by czy04 on 2024/5/10.
//

//modbus私有定义

#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#define MB_ADU_SIZE_MAX 256 // Maximum size of the ADU = address(1byte) + function code(1) + data(252bytes) + CRC(2bytes)
#define MB_ADU_SIZE_MIN 3   // Minimum size of the ADU
#define MB_PDU_SIZE_MAX 253 // Maximum size of the PDU = function code(1byte) + data(252bytes)
#define MB_PDU_SIZE_MIN 1   // Minimum size of the PDU
#define MB_PDU_OFF 1        // PDU offset in ADU
#define MB_FUNC_OFF 0   // Function code offset in PDU
#define MB_DATA_OFF 1   // Data offset in PDU

// Function codes
enum {
    MB_FC_READ_COILS = 1,
    MB_FC_READ_DISCRETE_INPUTS,
    MB_FC_READ_HOLDING_REGISTERS,
    MB_FC_READ_INPUT_REGISTERS,
    MB_FC_WRITE_SINGLE_COIL,
    MB_FC_WRITE_SINGLE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS,
    MB_FC_READWRITE_MULTIPLE_REGISTERS
};

// Exception codes
enum {
    MB_EX_OK = 0,
    MB_EX_ILLEGAL_FUNCTION,
    MB_EX_ILLEGAL_DATA_ADDRESS,
    MB_EX_ILLEGAL_DATA_VALUE,
    MB_EX_SLAVE_DEVICE_FAILURE,
    MB_EX_ACKNOWLEDGE,
    MB_EX_SLAVE_BUSY,
    MB_EX_MEMORY_PARITY_ERROR,
    MB_EX_GATEWAY_PATH_FAILED,
    MB_EX_GATEWAY_TGT_FAILED
};


#endif //MODBUS_PROTOCOL_H
