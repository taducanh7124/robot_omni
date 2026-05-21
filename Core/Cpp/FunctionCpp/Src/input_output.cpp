#include "input_output.hpp"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "usart.h"
#include "const.hpp"
#include "data.hpp"
#include "MotorControl.hpp"
#include "UART_DMA.hpp"

uint32_t tgNhanDuLieuCu = 0;  // Thời điểm cuối cùng nhận dữ liệu
uint32_t tgGuiDuLieuPiCu = 0; // Thời điểm cuối cùng gửi dữ liệu cho pi

// Bien toan cuc dung de xem trong iar
float v_robot = 0.0f;
float w_robot = 0.0f;
float theta_robot = 0.0f;
float vx = 0.0f;
float vy = 0.0f;
float v_fl = 0.0f;
float v_fr = 0.0f;
float v_rl = 0.0f;
float v_rr = 0.0f;
float ccr_fl = 0.0f;
float ccr_fr = 0.0f;
float ccr_rl = 0.0f;
float ccr_rr = 0.0f;
int count = 0;

void debug_nhanDuLieu()
{
    // TẮT KIÊM TRA > 1000 ĐỂ TRÁNH XÓA DỮ LIỆU
    // Dung robot neu khong gui du lieu trong 1 giay
    // if (HAL_GetTick() - tgNhanDuLieuCu > 1000)
    // {
    //     robot.motor_front_left.ccrTL = robot.motor_rear_left.ccrTL = 0;
    //     robot.motor_front_right.ccrTL = robot.motor_rear_right.ccrTL = 0;
    // }

    // Nếu chưa có cờ thì thoát tam thoi tat
    // if (!robot.state.isDataNew)
    //     return;

    // Hạ cờ
    // robot.state.isDataNew = false;

    if (hrxdata.isReadData == false)
    {
        // Tách dữ liệu từ uart bằng sscanf
        if (sscanf((char *)hrxdata.rxData, "%f,%f,%f", &v_robot, &w_robot, &theta_robot) == 3)
        {
            tgNhanDuLieuCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu

            // Giới hạn tốc độ tối đa
            if (v_robot > 30) // ccr
            {
                v_robot = 30.0f;
            }
            if (fabsf(w_robot) > 0.7f) // rad/s
            {
                w_robot = (w_robot > 0) ? 0.7f : -0.7f;
            }

            // 1. TÍNH VẬN TỐC vx, vy
            vx = v_robot * cosf(theta_robot);
            vy = v_robot * sinf(theta_robot);

            // 2. Tinh van toc tung banh
            v_fl = vx - vy - (w_robot * lxy);
            v_fr = vx + vy + (w_robot * lxy);
            v_rl = vx + vy - (w_robot * lxy);
            v_rr = vx - vy + (w_robot * lxy);

            // // 3. MAP TỪ m/s SANG CCR // tam thoi tat, tinh trong motor control // chua co du lieu thuc te
            // ccr_fl = fabsf(v_fl);
            // ccr_fr = fabsf(v_fr);
            // ccr_rl = fabsf(v_rl);
            // ccr_rr = fabsf(v_rr);

            // 4. GÁN giá trị vận tốc
            robot.motor_front_left.ccrTL = v_fl;
            robot.motor_rear_left.ccrTL = v_rl;
            robot.motor_front_right.ccrTL = v_fr;
            robot.motor_rear_right.ccrTL = v_rr;
        }
        hrxdata.isReadData = true; // Phất cờ báo có dữ liệu mới đã đọc xong
    }
}

// Hàm gửi dữ liệu odometry ra UART cho pi
void debug_guiDuLieu()
{
    // 1. Kiểm tra chu kỳ gửi (50ms = 20Hz)
    if (HAL_GetTick() - tgGuiDuLieuPiCu < 50)
    {
        return;
    }

    // 2. Kiểm tra xem bộ DMA đã rảnh chưa (1 = Rảnh, 0 = Đang bận gửi gói cũ)
    if (!robot.state.isSendDataNew)
    {
        return;
    }

    tgGuiDuLieuPiCu = HAL_GetTick();

    // 3. Đóng gói dữ liệu (Dùng luôn mảng txBuffer toàn cục đã khai báo ở UART_DMA.hpp)
    // Định dạng: odom_x, odom_y, odom_theta, odom_vx, 0.000, odom_w \n
    static int doDaiGoiTin;

    doDaiGoiTin = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                           odom_x, odom_y, odom_theta_rad, odom_vx, odom_vy, odom_w_rad);

    // 4. Kích hoạt DMA gửi đi
    if (doDaiGoiTin > 0)
    {
        // Hạ cờ BẬN ngay lập tức để khóa hàm lại cho chu kỳ sau
        robot.state.isSendDataNew = 0;

        // Đẩy dữ liệu đi (DMA sẽ tự động phất cờ rảnh ở ngắt TxCpltCallback)
        HAL_UART_Transmit_DMA(&huart6, txBuffer, doDaiGoiTin);
    }
}