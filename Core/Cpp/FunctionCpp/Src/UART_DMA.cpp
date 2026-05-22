#include "UART_DMA.hpp"
#include <string.h>
#include "MotorControl.hpp"

UART_DMA UART_DMA_6;
RXData_HandleTypeDef hrxdata;
uint8_t rxBuffer[200];
uint8_t txBuffer[200];

void UART_DMA::init(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t size)
{
    _huart = huart;
    _rxBuffer = rxBuffer;
    _size = size;

    hrxdata.isReadData = true; // Ban đầu chưa có dữ liệu mới, nên đặt cờ là đã đọc xong (true)

    // Kích hoạt DMA lần đầu tiên
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rxBuffer, _size);
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}

extern "C"
{
    // Hàm này tự động gọi khi nhận xong 1 gói tin hoặc đường truyền IDLE
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
    {
        if (huart->Instance == USART6)
        {
            if (hrxdata.isReadData == true)
            {
                // Bảo vệ tràn mảng và ép size an toàn trước khi xử lý
                if (size < sizeof(rxBuffer))
                {
                    rxBuffer[size] = '\0';
                }
                else
                {
                    // Khi mảng đầy (size >= 200), ép size về 199 để chốt \0 ở ô cuối cùng
                    size = sizeof(rxBuffer) - 1;
                    rxBuffer[size] = '\0';
                }

                // Bây giờ size + 1 tối đa sẽ chỉ là 200, an toàn tuyệt đối cho cả 2 mảng!
                memcpy(hrxdata.rxData, rxBuffer, size + 1);

                hrxdata.isReadData = false;   // Tắt cờ, báo là có dữ liệu mới chưa đọc
                robot.state.isDataNew = true; // Phất cờ báo cho hàm xử lý
            }
            // // Ép dừng trạng thái Rx cũ trước khi gọi lại để tránh kẹt HAL_BUSY
            // HAL_UART_AbortReceive(huart); // tat di?

            // Tái kích hoạt DMA cho lần nhận tiếp theo
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }

    // Hàm ngắt xử lí khi uart dma bị lỗi
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART6)
        {
            // Nếu có lỗi (Nhiễu, Overrun, Lệch Baudrate...), kéo chip ra khỏi lỗi và khởi động lại DMA
            HAL_UART_AbortReceive(huart);
            // Tái kích hoạt DMA cho lần nhận tiếp theo
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer, sizeof(rxBuffer));
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            hrxdata.isReadData = true;
        }
    }

    // Hàm này tự động gọi khi gửi xong (TX)
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART6)
        {
            robot.state.isSendDataNew = 1;
        }
    }
}