#include "main_cpp.hpp"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "led_main.hpp"
#include "input_output.hpp"
#include "MotorControl.hpp"
#include "data.hpp"
#include "uart_ring_buffer.h"
#include "pid_controller.hpp"

// Khai báo mảng vật lý và Object Ring Buffer
#define RX_BUF_SIZE 256
uint8_t dma_rx_buffer[RX_BUF_SIZE];
RingBuffer_t ros2_uart;
// Mảng tạm chứa chuỗi hoàn chỉnh
char line[64];

// Hàm Parse Float (đã viết ở câu trước)
extern bool Parse_Robot_Command_Float(const char *str);

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

    // timer cho PID
    HAL_TIM_Base_Start_IT(&htim10);
}
void khoiTaoSerial()
{
    RingBuffer_Init(&ros2_uart, &huart6, dma_rx_buffer, RX_BUF_SIZE);
}

uint32_t testTimer = 0;
uint32_t timerCount = 0;

uint32_t timeMain = 0;
uint32_t testTimerMain = 0;
uint32_t loopCount = 0;

void main_cpp()
{
    khoiTaoMPU();
    khoiTaoEncoder();
    khoiTaoMotor();
    khoiTaoSerial();

    uint32_t tgNhayLedCu = 0;
    uint32_t tgDieuKhienMotorCu = 0;

    // while (1)
    // {
    //     HAL_UART_Transmit_DMA(&huart6, (uint8_t *)"Robot Omni Started!\n", 21);
    // }

    while (1)
    {
        // số lần gọi while(1) trong 1s
        loopCount++;
        if (HAL_GetTick() - timeMain > 1000)
        {
            timeMain = HAL_GetTick();
            testTimerMain = loopCount;
            loopCount = 0;
        }

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

            // số lần tính toán
            timerCount++;
            static uint32_t time = 0;
            if (HAL_GetTick() - time > 1000)
            {
                time = HAL_GetTick();
                testTimer = timerCount;
                timerCount = 0;
            }
        }

        // Nhan du lieu dieu khien dao vao
        // debug_nhanDuLieu();

        // 3. Liên tục gọi hàm ReadUntil.
        // Hàm này tự động chờ cho đến khi nhận đủ 1 chuỗi kết thúc bằng '\n'
        if (RingBuffer_ReadUntil(&ros2_uart, line, sizeof(line), '\n') > 0)
        {

            // Xử lý chuỗi ngay lập tức
            if (Parse_Robot_Command_Float(line))
            {
                // Đã cập nhật thành công robot_vx, robot_vy, robot_theta
                // Chạy hàm điều khiển động cơ tại đây...
            }
        }

        // Dieu khien dong co du tren du lieu dau vao
        if (HAL_GetTick() - tgDieuKhienMotorCu >= 10)
        {
            tgDieuKhienMotorCu = HAL_GetTick();
            controlOnDinhCCR();
        }

        // TÍNH TOÁN ODOMETRY
        tinhOdom();

        // Gui du lieu odometry ra uart
        debug_guiDuLieu();
    }
}

extern "C"
{
    // uint32_t testTimer_mpu = 0;
    // uint32_t timerCount_mpu = 0;
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == IMU_EXTI_Pin)
        {
            isDMPNewData = true;

            // // số lần ngắt /1s
            // timerCount_mpu++;
            // static uint32_t time = 0;
            // if (HAL_GetTick() - time > 1000)
            // {
            //     time = HAL_GetTick();
            //     testTimer_mpu = timerCount_mpu;
            //     timerCount_mpu = 0;
            // }
        }
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if (htim->Instance == TIM10)
        {
            tinhTargetWheelSpeeds_c(); // Tính toán tốc độ bánh xe mục tiêu mỗi 10ms
        }
    }
}