//
// Created by czy04 on 2024/5/10.
//

#include "modbus_RTU.h"

void modbus_frontend::Handle() {
    uint8_t len;
    len = recv();
    if (len > 0) {
        ctx->rx_len = len;
        memcpy(ctx->rx_data, rx_buff, len);
        send(ctx->tx_data, ctx->tx_len);
    }
}

uint8_t modbus_frontend::recv() {
    uint8_t cnt = MAX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buff, MB_ADU_SIZE_MAX);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}