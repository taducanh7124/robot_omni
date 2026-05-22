#ifndef UART_DMA_HPP
#define UART_DMA_HPP

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
class UART_DMA
{
private:
    UART_HandleTypeDef *_huart;
    uint8_t *_rxBuffer;
    uint16_t _size;

public:
    // Constructor mặc định
    UART_DMA()
    {
        _huart = nullptr;
        _rxBuffer = nullptr;
        _size = 0;
    };

    // Hàm khởi tạo
    void init(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t size);
};

typedef struct
{
    uint8_t rxData[200];
    bool isReadData;
} HandleRXData;

extern HandleRXData hrxdata;

// khai bao UART DMA
extern UART_DMA UART_DMA_6;
extern uint8_t rxBuffer[200];
extern uint8_t txBuffer[200];
#endif

#endif