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
bool isCheckVel = false;
uint32_t tgCheckVelCu = 0;

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

    if (hrxdata.isReadData == true) // Neu du lieu da duoc xu li thì thoat
        return;

    // Tách dữ liệu từ uart bằng sscanf
    if (sscanf((char *)hrxdata.rxData, "%f,%f,%f", &v_robot, &w_robot, &theta_robot) == 3)
    {
        tgNhanDuLieuCu = HAL_GetTick(); // Cập nhật thời điểm nhận dữ liệu

        // Giới hạn tốc độ tối đa
        if (v_robot > 0.5f) // m/s
        {
            v_robot = 0.5f;
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

        // 4. GÁN giá trị vận tốc
        robot.motor_front_left.vanTocDich = v_fl;
        robot.motor_rear_left.vanTocDich = v_rl;
        robot.motor_front_right.vanTocDich = v_fr;
        robot.motor_rear_right.vanTocDich = v_rr;
    }
    hrxdata.isReadData = true; // Phất cờ báo có dữ liệu mới đã đọc xong, ke ca co loi khong doc duoc
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

// Chương trình kiểm tra vận tốc
    // // Nếu bạn vừa bật cờ test bằng tay trong Debugger
    // if (isCheckVel == true) 
    // {
    //     // 1. Chốt mốc thời gian ngay khoảnh khắc đầu tiên
    //     if (tgCheckVelCu == 0) {
    //         tgCheckVelCu = HAL_GetTick(); 
    //     }

    //     // 2. Kiểm tra xem đã chạy hết 3000ms (3 giây) chưa?
    //     if (HAL_GetTick() - tgCheckVelCu <= 7000) 
    //     {
    //         v_robot = 0.1f; // Bơm vận tốc 0.1 m/s
    //     } 
    //     else 
    //     {
    //         // 3. Hết 3 giây: Dừng xe và tự động khóa cờ lại
    //         v_robot = 0.0f;
    //         isCheckVel = false; // Tự tắt cờ để không chạy nữa
    //         tgCheckVelCu = 0;   // Reset mốc thời gian cho lần test sau
    //     }
    // } 
    // else 
    // {
    //     // Nếu cờ đang tắt, đảm bảo mốc thời gian luôn bằng 0 chờ lệnh
    //     tgCheckVelCu = 0;
        
    //     // Lưu ý: Không gán v_robot = 0.0f ở đây.
    //     // Vì nếu bạn gán 0 ở đây, nó sẽ đè lên lệnh điều khiển thật từ ESP32/Tay cầm.
    // }