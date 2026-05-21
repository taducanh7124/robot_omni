#include "data.hpp"
#include <math.h>
#include "main.h"
#include "main_cpp.hpp"
#include "tim.h"
#include "const.hpp"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "MotorControl.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Các biến Odometry
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta_rad = 0.0f;
float odom_theta_deg = 0.0f;

float odom_vx = 0.0f;
float odom_vy = 0.0f;
float odom_w_rad = 0.0f;

float vtTrungBinhTrai = 0;
float vtTrungBinhPhai = 0;

float delta_sx = 0.0f;
float delta_sy = 0.0f;
float odom_delta_xx = 0.0f;
float odom_delta_xy = 0.0f;
float odom_delta_yx = 0.0f;
float odom_delta_yy = 0.0f;

uint32_t tgTinhOdomCu = 0;

// Biến cho MPU6050 DMP
MPU6050 mpu;
bool isDMPReady = false;
volatile bool isDMPNewData = false;
uint8_t fifoBuffer[64];

Quaternion quaternion;
VectorFloat gravity;
float mpu_theta_ypr_rad[3];
int16_t mpu_w_gyro_raw[3];

// Hàm khởi tạo MPU6050 và DMP
void khoiTaoMPU()
{
    mpu.initialize();
    if (mpu.testConnection() && mpu.dmpInitialize() == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        isDMPReady = true;
    }
}

// Tính vận tốc tịnh tiến vx và vận tốc ngang vy
void tinhVanToc(float delta_t)
{
    // Xung qua khu cua banhh, dùng static để lưu giá trị cũ khi kết thúc hàm
    static int32_t xungQK_FR = 0;
    static int32_t xungQK_RL = 0;
    static int32_t xungQK_RR = 0;
    static int32_t xungQK_FL = 0;

    // Xung hien tai cua banh
    int32_t xungHT_FL = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    int32_t xungHT_FR = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t xungHT_RR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t xungHT_RL = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    // Chenh lech xung giua 2 lan do
    int32_t delta_FL = xungHT_FL - xungQK_FL;
    int32_t delta_FR = xungHT_FR - xungQK_FR;
    int32_t delta_RL = xungHT_RL - xungQK_RL;
    int32_t delta_RR = xungHT_RR - xungQK_RR;

    // Xử lý tràn 16-bit
    if (delta_FL > 32768)
        delta_FL -= 65536;
    else if (delta_FL < -32768)
        delta_FL += 65536;

    if (delta_FR > 32768)
        delta_FR -= 65536;
    else if (delta_FR < -32768)
        delta_FR += 65536;

    if (delta_RL > 32768)
        delta_RL -= 65536;
    else if (delta_RL < -32768)
        delta_RL += 65536;

    if (delta_RR > 32768)
        delta_RR -= 65536;
    else if (delta_RR < -32768)
        delta_RR += 65536;

    // Xác định dấu encoder do có 1 kênh, dùng dấu từ đièu khiển hướng động cơ
    if (robot.motor_front_left.dir == 0)
        delta_FL = -delta_FL;
    if (robot.motor_front_right.dir == 0)
        delta_FR = -delta_FR;
    if (robot.motor_rear_left.dir == 0)
        delta_RL = -delta_RL;
    if (robot.motor_rear_right.dir == 0)
        delta_RR = -delta_RR;

    // Tính vận tốc của từng bánh
    robot.motor_front_left.vanToc = (delta_FL * MET1XUNG) / delta_t;
    robot.motor_rear_left.vanToc = (delta_RL * MET1XUNG) / delta_t;
    robot.motor_front_right.vanToc = (delta_FR * MET1XUNG) / delta_t;
    robot.motor_rear_right.vanToc = (delta_RR * MET1XUNG) / delta_t;

    // Đưa vào biến cục bộ cho công thức dễ nhìn
    float v_fl = robot.motor_front_left.vanToc;
    float v_rl = robot.motor_rear_left.vanToc;
    float v_fr = robot.motor_front_right.vanToc;
    float v_rr = robot.motor_rear_right.vanToc;

    // Tính vận tốc bằng mô hình động học thuận
    odom_vx = (v_fl + v_fr + v_rl + v_rr) / 4.0f;
    // Trượt sang trái là dương, trượt sang phải là âm
    odom_vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0f;

    // Cập nhật giá trị encoder cũ
    xungQK_FL = xungHT_FL;
    xungQK_FR = xungHT_FR;
    xungQK_RL = xungHT_RL;
    xungQK_RR = xungHT_RR;
}

// Tính vận tốc góc z và góc z, gọi trong main_cpp.cpp để không tràn bộ đệm FIFO của MPU6050
void tinhThongSoGoc()
{
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(mpu_theta_ypr_rad, &quaternion, &gravity);
        mpu.dmpGetGyro(mpu_w_gyro_raw, fifoBuffer);

        odom_theta_rad = mpu_theta_ypr_rad[0];
        odom_theta_deg = odom_theta_rad * (180.0f / M_PI);

        odom_w_rad = (mpu_w_gyro_raw[2] / 16.4f) * (M_PI / 180.0f);
    }
}

// Tính tọa độ x, y
void tinhToaDo(float delta_t)
{
    // Tính quãng đường đi được trong delta t theo 2 trục
    delta_sx = odom_vx * delta_t;
    delta_sy = odom_vy * delta_t;

    // Tính tọa độ khi robot di chuyển theo từng trục
    // Truc x, giá trị tọa độ x thay đổi khi robot đi thẳng, trục x robot, và đi ngang, trục y robot
    odom_delta_xx = delta_sx * cosf(odom_theta_rad);
    odom_delta_xy = delta_sy * sinf(odom_theta_rad);
    // Truc y, giá trị tọa độ y thay đổi khi robot đi thẳng, trục x robot, và đi ngang, trục y robot
    odom_delta_yx = delta_sx * sinf(odom_theta_rad);
    odom_delta_yy = delta_sy * cosf(odom_theta_rad);

    // Tính tọa độ tổng bằng ma trận xoay
    // Di thang truc x robot thì tang x toan cuc, di ngang trai truc y robot thì giam x toàn cục
    odom_x += (odom_delta_xx - odom_delta_xy);
    // Di thang truc x robot thì tang y toan cuc, di ngang trai truc y robot thì tang y toàn cục
    odom_y += (odom_delta_yx + odom_delta_yy);
}

// Hàm tính odometry
void tinhOdom()
{
    uint32_t tgTinhOdomMoi = HAL_GetTick();

    if (tgTinhOdomMoi - tgTinhOdomCu < 100)
    {
        return;
    }

    float delta_t = (tgTinhOdomMoi - tgTinhOdomCu) / 1000.0f;
    tgTinhOdomCu = tgTinhOdomMoi;

    // Tính toán quãng đường, vận tốc bánh xe
    tinhVanToc(delta_t);

    // Tích phân ra tọa độ x, y
    tinhToaDo(delta_t);
}