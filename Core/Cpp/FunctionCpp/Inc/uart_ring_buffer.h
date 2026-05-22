//#ifndef UART_RING_BUFFER_H
//#define UART_RING_BUFFER_H
//
//#include <stdint.h>
//#include <stdbool.h>
//
//// thư viện phần cứng
//#include "stm32f4xx_hal.h"
//
//typedef struct
//{
//    UART_HandleTypeDef *huart; // Con trỏ trỏ đến cổng UART (vd: &huart1)
//    uint8_t *buffer;           // Con trỏ trỏ đến mảng vật lý
//    uint16_t size;             // Kích thước của mảng
//    uint16_t tail;             // Con trỏ đọc (phần mềm quản lý)
//} RingBuffer_t;
//
///**
// * @brief Khởi tạo Ring Buffer và kích hoạt DMA Circular
// */
//void RingBuffer_Init(RingBuffer_t *rb, UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
//
///**
// * @brief Kiểm tra số lượng byte chưa đọc trong buffer
// */
//uint16_t RingBuffer_GetPending(RingBuffer_t *rb);
//
///**
// * @brief Đọc 1 byte từ Ring Buffer
// * @return true nếu có dữ liệu, false nếu buffer rỗng
// */
//bool RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *out_data);
//
///**
// * @brief Đọc liên tục cho đến khi gặp ký tự phân tách (vd: '\n')
// * @return Số lượng byte đọc được. Trả về 0 nếu chưa nhận đủ 1 khung.
// */
//uint16_t RingBuffer_ReadUntil(RingBuffer_t *rb, char *out_str, uint16_t max_len, char delimiter);
//
//#endif // UART_RING_BUFFER_H

#ifndef UART_RINGBUFFER_H
#define UART_RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" 

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UART_HandleTypeDef *huart; 
    uint8_t *buffer;           
    uint16_t size;             
    uint16_t tail;             
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
uint16_t RingBuffer_GetPending(RingBuffer_t *rb);
bool RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *out_data);
uint16_t RingBuffer_ReadUntil(RingBuffer_t *rb, char *out_str, uint16_t max_len, char delimiter);

#ifdef __cplusplus
}
#endif

#endif /* UART_RINGBUFFER_H */