#include "main_cpp.hpp"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "led_main.hpp"
#include "UART_DMA.hpp"
#include "input_output.hpp"
#include "MotorControl.hpp"
#include "data.hpp"

void khoiTaoEncoder()
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);
}
void khoiTaoMotor()
{
    MotorCtr_FL.init(&htim5, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin);
    MotorCtr_FR.init(&htim5, TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin);
    MotorCtr_RR.init(&htim5, TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin);
    MotorCtr_RL.init(&htim5, TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin);
}
void khoiTaoSerial()
{
    UART_DMA_6.init(&huart6, rxBuffer, sizeof(rxBuffer));
}
void khoiTaoPID() {
    // Khởi tạo cho bánh trước trái
    robot.motor_front_left.pid.kp = 0.0f; 
    robot.motor_front_left.pid.ki = 0.0f; 
    robot.motor_front_left.pid.saiSoCongDon = 0.0f;
    robot.motor_front_left.pid.saiSoCu = 0.0f;
    
    robot.motor_front_right.pid.kp = 0.0f; 
    robot.motor_front_right.pid.ki = 0.0f; 
    robot.motor_front_right.pid.saiSoCongDon = 0.0f;
    robot.motor_front_right.pid.saiSoCu = 0.0f;

    robot.motor_rear_left.pid.kp = 0.0f; 
    robot.motor_rear_left.pid.ki = 0.0f; 
    robot.motor_rear_left.pid.saiSoCongDon = 0.0f;
    robot.motor_rear_left.pid.saiSoCu = 0.0f;

    robot.motor_rear_right.pid.kp = 0.0f; 
    robot.motor_rear_right.pid.ki = 0.0f; 
    robot.motor_rear_right.pid.saiSoCongDon = 0.0f;
    robot.motor_rear_right.pid.saiSoCu = 0.0f;
}

void main_cpp()
{
    khoiTaoMPU();
    khoiTaoEncoder();
    khoiTaoMotor();
    khoiTaoSerial();
    khoiTaoPID();

    uint32_t tgNhayLedCu = 0;
    uint32_t tgDieuKhienMotorCu = HAL_GetTick();
    const float DELTA_T = 0.05f;

    while (1)
    {
        // Nháy LED báo trạng thái
        if (HAL_GetTick() - tgNhayLedCu >= 1000)
        {
            tgNhayLedCu = HAL_GetTick();
            nhayLed();
        }

        // ĐỌC IMU NGAY KHI CÓ NGẮT, đảm bảo FIFO không bao giờ bị tràn và gây lỗi
        if (isDMPReady && isDMPNewData)
        {
            isDMPNewData = false; // Xóa cờ ngắt
            tinhThongSoGoc();     // Cập nhật ngay lập tức góc Yaw và Vận tốc góc yaw
        }

        // Nhan du lieu dieu khien dau vao
        debug_nhanDuLieu();

        // Dieu khien dong co du tren du lieu dau vao
        if (HAL_GetTick() - tgDieuKhienMotorCu >= 10)
        {
            tgDieuKhienMotorCu = HAL_GetTick();
            tinhVanToc(DELTA_T);
            dieuKhienMotorPID(DELTA_T);
        }


        // TÍNH TOÁN ODOMETRY
        tinhOdom();

        // Gui du lieu odometry ra uart
        debug_guiDuLieu();
    }
}

extern "C"
{
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == IMU_EXTI_Pin)
        {
            isDMPNewData = true;
        }
    }
}