#include "main_cpp.hpp"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "led_main.hpp"
#include "input_output.hpp"
#include "MotorControl.hpp"
#include "data.hpp"
#include "uart_ring_buffer.h"

#include "PID_AutoTune_v0.h"
#include "PID_v1.h"
#include <math.h> // Để sử dụng hàm round()


// Khai báo mảng vật lý và Object Ring Buffer
#define RX_BUF_SIZE 1024
uint8_t dma_rx_buffer[RX_BUF_SIZE];
RingBuffer_t ros2_uart;
// Mảng tạm chứa chuỗi hoàn chỉnh
char line[64];

double Input_FL = 0, Output_FL = 0;
double Input_FR = 0, Output_FR = 0;
double Input_RL = 0, Output_RL = 0;
double Input_RR = 0, Output_RR = 0;

// Khai báo chung một bộ nhớ Input, Output cho cả 2 class, PID cu 12, 50, 0.001
PID PID_FL(&Input_FL, &Output_FL, &robot.motor_front_left.ccrPositive, 13, 50, 0.01, PID::P_ON_M, PID::DIRECT);
PID PID_FR(&Input_FR, &Output_FR, &robot.motor_front_right.ccrPositive, 13, 50, 0.01, PID::P_ON_M, PID::DIRECT);
PID PID_RL(&Input_RL, &Output_RL, &robot.motor_rear_left.ccrPositive, 13, 50, 0.01, PID::P_ON_M, PID::DIRECT);
PID PID_RR(&Input_RR, &Output_RR, &robot.motor_rear_right.ccrPositive, 13, 50, 0.01, PID::P_ON_M, PID::DIRECT);

void PID_setup()
{
    PID_FL.SetOutputLimits(0, 250); // Giới hạn output của PID (tương ứng với CCR)
    PID_FL.SetMode(PID::AUTOMATIC); // Chạy PID ở chế độ tự động
    PID_FL.SetSampleTime(30);       // Thời gian mẫu 30ms (tương ứng với vòng lặp 10Hz)

    PID_FR.SetOutputLimits(0, 250); // Giới hạn output của PID (tương ứng với CCR)
    PID_FR.SetMode(PID::AUTOMATIC); // Chạy PID ở chế độ tự động
    PID_FR.SetSampleTime(30);       // Thời gian mẫu 30ms (tương ứng với vòng lặp 10Hz)

    PID_RL.SetOutputLimits(0, 250); // Giới hạn output của PID (tương ứng với CCR)
    PID_RL.SetMode(PID::AUTOMATIC); // Chạy PID ở chế độ tự động
    PID_RL.SetSampleTime(30);       // Thời gian mẫu 30ms   (tương ứng với vòng lặp 10Hz)

    PID_RR.SetOutputLimits(0, 250); // Giới hạn output của PID (tương ứng với CCR)
    PID_RR.SetMode(PID::AUTOMATIC); // Chạy PID ở chế độ tự động
    PID_RR.SetSampleTime(30);       // Thời gian mẫu 30ms (tương ứng với vòng lặp 10Hz)
}
uint32_t timeloop = 30; // Thời gian giữa các lần tính toán PID (ms)

// Hàm chuyển đổi an toàn từ double sang PWM (0-255)
uint8_t doubleToPWM(double output_val) 
{
    // 1. Làm tròn số thực tới số nguyên gần nhất
    int pwm = (int)round(output_val);
    
    // 2. Kẹp rào chắn an toàn tuyệt đối
    if (pwm > 255) return 255;
    if (pwm < 0) return 0;
    
    // 3. Trả về kiểu số nguyên không dấu 8-bit
    return (uint8_t)pwm;
}

void PID_Loop()
{
    uint32_t now = HAL_GetTick();
    static uint32_t last_vel_calc_time = 0;
    if (now - last_vel_calc_time >= timeloop)
    {
        // --- 1. BIẾN LƯU TRẠNG THÁI ---
        static int32_t xungQK_FL = 0;
        static int32_t raw_delta_FL = 0;

        static int32_t xungQK_FR = 0;
        static int32_t raw_delta_FR = 0;

        static int32_t xungQK_RL = 0;
        static int32_t raw_delta_RL = 0;

        static int32_t xungQK_RR = 0;
        static int32_t raw_delta_RR = 0;

        // =========================================================
        // CHỈ TÍNH VẬN TỐC MỖI 20MS (Tránh việc delta luôn bằng 0)
        // =========================================================

        int32_t xungHT_FL = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
        int32_t xungHT_FR = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t xungHT_RL = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        int32_t xungHT_RR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

        raw_delta_FL = xungHT_FL - xungQK_FL;
        raw_delta_FR = xungHT_FR - xungQK_FR;
        raw_delta_RL = xungHT_RL - xungQK_RL;
        raw_delta_RR = xungHT_RR - xungQK_RR;

        // Xử lý tràn Timer 16-bit
        if (raw_delta_FL > 32768)
            raw_delta_FL -= 65536;
        else if (raw_delta_FL < -32768)
            raw_delta_FL += 65536;

        // Xử lý tràn Timer 16-bit
        if (raw_delta_FR > 32768)
            raw_delta_FR -= 65536;
        else if (raw_delta_FR < -32768)
            raw_delta_FR += 65536;

        // Xử lý tràn Timer 16-bit
        if (raw_delta_RL > 32768)
            raw_delta_RL -= 65536;
        else if (raw_delta_RL < -32768)
            raw_delta_RL += 65536;

        // Xử lý tràn Timer 16-bit
        if (raw_delta_RR > 32768)
            raw_delta_RR -= 65536;
        else if (raw_delta_RR < -32768)
            raw_delta_RR += 65536;

        xungQK_FL = xungHT_FL;
        xungQK_FR = xungHT_FR;
        xungQK_RL = xungHT_RL;
        xungQK_RR = xungHT_RR;

        // Cập nhật Input liên tục cho TẤT CẢ các bộ điều khiển
        Input_FL = (double)raw_delta_FL;
        Input_FR = (double)raw_delta_FR;
        Input_RL = (double)raw_delta_RL;
        Input_RR = (double)raw_delta_RR;

        // =========================================================
        // KHỐI ĐIỀU KHIỂN PID & AUTOTUNE
        // =========================================================
        // Trạng thái 2: Hệ thống chạy PID bình thường
        PID_FL.Compute(now);
        PID_FR.Compute(now);
        PID_RL.Compute(now);
        PID_RR.Compute(now);

        // =========================================================
        // XỬ LÝ SỐ ÂM ĐỂ XUẤT RA ĐỘNG CƠ AN TOÀN
        // =========================================================

        MotorCtr_FL.control(doubleToPWM(Output_FL), (MotorDir)robot.motor_front_left.dir);
        MotorCtr_FR.control(doubleToPWM(Output_FR), (MotorDir)robot.motor_front_right.dir);
        MotorCtr_RL.control(doubleToPWM(Output_RL), (MotorDir)robot.motor_rear_left.dir);
        MotorCtr_RR.control(doubleToPWM(Output_RR), (MotorDir)robot.motor_rear_right.dir);

        last_vel_calc_time = now;
    }
}

bool isTuning = false;
double P = 0, I = 0, D = 0;
void tuningPID()
{
    if (!isTuning)
        return;
    PID_FL.SetTunings(P, I, D);
}

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

    // // timer cho PID
    // HAL_TIM_Base_Start_IT(&htim10);
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
    PID_setup();

    uint32_t tgNhayLedCu = 0;
    uint32_t tgDieuKhienMotorCu = 0;

    while (1)
    {
        tuningPID();
        PID_Loop(); // Chạy hàm PID trong vòng lặp chính

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
        // if (HAL_GetTick() - tgDieuKhienMotorCu >= 10)
        // {
        //     tgDieuKhienMotorCu = HAL_GetTick();
        //     controlOnDinhCCR();
        // }

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
        }
    }
}