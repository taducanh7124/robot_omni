#ifndef PIN_CONFIG_HPP
#define PIN_CONFIG_HPP

#include <stdint.h>
#include "main.h"

struct PinConfig_Out
{
    volatile uint32_t *Reg_BSRR; // Nơi lưu địa chỉ thanh ghi để BẬT (Set) hoặc TẮT (Reset) Bit
    volatile uint32_t *Reg_ODR;  // Nơi lưu địa chỉ thanh ghi để ĐỌC (Output Data Register)
    volatile uint32_t *Reg_IDR;  // Nơi lưu địa chỉ thanh ghi để ĐỌC (Input Data Register)
    uint32_t pinMask;

    // Hàm khởi tạo, truyền vào port và pin để lưu địa chỉ thanh ghi tương ứng
    // GPIO_TypeDef* port; // Ví dụ: GPIOA, GPIOB
    // uint16_t pin;       // Ví dụ: GPIO_PIN_0 = 0x0001, GPIO_PIN_1 = 0x0002
    void init(GPIO_TypeDef *port, uint16_t pin)
    {
        Reg_BSRR = &(port->BSRR); // Lưu địa chỉ thanh ghi BSRR của port đó
        Reg_ODR = &(port->ODR);   // Lưu địa chỉ thanh ghi ODR của port đó
        Reg_IDR = &(port->IDR);   // Lưu địa chỉ thanh ghi IDR của port đó
        pinMask = pin;            // Lưu vị trí chân ví dụ
    }
    // HIGH
    inline void high() const
    {
        *Reg_BSRR = pinMask; // Ghi thẳng vào địa chỉ đã lưu -> Tốc độ cực nhanh
    }
    // LOW
    inline void low() const
    {
        *Reg_BSRR = pinMask << 16u; // Ghi thẳng vào địa chỉ đã lưu -> Tốc độ cực nhanh
    }
};

struct PinConfig_In
{
    volatile uint32_t *Reg_IDR; // Nơi lưu địa chỉ thanh ghi để ĐỌC (Input Data Register)
    uint32_t pinMask;

    // Hàm khởi tạo
    void init(GPIO_TypeDef *port, uint16_t pin)
    {
        Reg_IDR = &(port->IDR); // Lưu địa chỉ thanh ghi IDR của port đó
        pinMask = pin;          // Lưu vị trí chân
    }

    // ĐỌC TRẠNG THÁI
    inline bool read() const
    {
        return ((*Reg_IDR) & pinMask) != 0; // Trả về true nếu chân ở mức cao, false nếu mức thấp
    }
};

inline uint32_t generate_lut(uint16_t GPIO_Pin, bool isHigh)
{
    if (isHigh)
    {
        return (uint32_t)GPIO_Pin; // Bit thấp: SET
    }
    else
    {
        return (uint32_t)GPIO_Pin << 16u; // Bit cao: RESET
    }
}
#endif