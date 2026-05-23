#include "uart_ring_buffer.h"

// Hàm nội bộ (Private) để tính con trỏ Head từ thanh ghi DMA
static uint16_t RingBuffer_GetHead(RingBuffer_t *rb) {
    // __HAL_DMA_GET_COUNTER trả về số data CÒN LẠI cần chuyển
    return rb->size - __HAL_DMA_GET_COUNTER(rb->huart->hdmarx);
}

void RingBuffer_Init(RingBuffer_t *rb, UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size) {
    rb->huart = huart;
    rb->buffer = buf;
    rb->size = size;
    rb->tail = 0;

    // Kích hoạt UART nhận dữ liệu bằng DMA (bắt buộc cấu hình DMA Circular trong CubeMX)
    HAL_UART_Receive_DMA(rb->huart, rb->buffer, rb->size);
}

uint16_t RingBuffer_GetPending(RingBuffer_t *rb) {
    uint16_t head = RingBuffer_GetHead(rb);
    
    if (head >= rb->tail) {
        return head - rb->tail;
    } else {
        // Trường hợp Head đã đi qua điểm nối vòng (Wrap-around)
        return (rb->size - rb->tail) + head;
    }
}

bool RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *out_data) {
    uint16_t head = RingBuffer_GetHead(rb);
    
    if (rb->tail == head) {
        return false; // Buffer rỗng
    }
    
    *out_data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size; // Tăng Tail, tự động vòng về 0
    
    return true;
}

uint16_t RingBuffer_ReadUntil(RingBuffer_t *rb, char *out_str, uint16_t max_len, char delimiter) {
    uint16_t head = RingBuffer_GetHead(rb);
    uint16_t count = 0;
    
    // Nếu không có dữ liệu, thoát luôn
    if (rb->tail == head) return 0;
    
    // Quét tìm xem ký tự delimiter (như '\n') có tồn tại trong đoạn chưa đọc không
    uint16_t temp_tail = rb->tail;
    bool found = false;
    
    while (temp_tail != head) {
        if (rb->buffer[temp_tail] == delimiter) {
            found = true;
            break;
        }
        temp_tail = (temp_tail + 1) % rb->size;
    }
    
    // Nếu không tìm thấy dấu kết thúc, chưa làm gì cả (chờ ROS2 gửi nốt)
    if (!found) return 0;
    
    // Nếu đã tìm thấy, tiến hành bốc dữ liệu thật sự ra ngoài
    while (rb->tail != head) {
        uint8_t c = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->size;
        
        if (c == delimiter) {
            out_str[count] = '\0'; // Chốt chuỗi
            return count;          // Trả về số byte hợp lệ
        } 
        else {
            if (count < max_len - 1) {
                out_str[count++] = c;
            }
        }
    }
    return count;
}