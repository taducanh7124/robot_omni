#ifndef DATA_HPP
#define DATA_HPP

#include "main.h"
#include <stdint.h>

// Tọa độ và góc
extern float odom_x;
extern float odom_y;
extern float odom_theta_rad;
extern float odom_theta_deg;

// Vận tốc và vận tốc góc
extern float odom_vx;
extern float odom_vy;
extern float odom_w_rad;

// Các biến liên quan đến tính toán odometry
extern bool isDMPReady;
extern volatile bool isDMPNewData;
extern uint32_t tgTinhOdomCu;

// Hàm tính toán odometry
void khoiTaoMPU(void);
void tinhVanToc(float delta_t);
void tinhThongSoGoc(void);
void tinhOdom(void);

#endif