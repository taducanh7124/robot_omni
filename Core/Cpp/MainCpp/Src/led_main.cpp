#include "led_main.hpp"
#include "main.h"

void nhayLed()
{
    LED_MAIN_GPIO_Port->ODR ^= LED_MAIN_Pin; // Đảo trạng thái của chân LED
}