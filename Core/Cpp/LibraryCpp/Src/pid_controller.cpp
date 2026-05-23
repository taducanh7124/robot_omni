#include "pid_controller.hpp"
// Sửa lại cấu hình PID: Cho phép output từ -250 đến 250
PIDController pid_motor_fl(1.5f, 0.2f, 0.05f, 250.0f, -250.0f); 
PIDController pid_motor_fr(1.5f, 0.2f, 0.05f, 250.0f, -250.0f);
PIDController pid_motor_rl(1.5f, 0.2f, 0.05f, 250.0f, -250.0f);
PIDController pid_motor_rr(1.5f, 0.2f, 0.05f, 250.0f, -250.0f);

volatile float target_w_fl = 0.0f;
volatile float target_w_fr = 0.0f;
volatile float target_w_rl = 0.0f;
volatile float target_w_rr = 0.0f;

void tinhTargetWheelSpeeds()
{
    // --- 1. Biến lưu trạng thái quá khứ ---
    static int32_t xungQK_FL = 0;
    static int32_t xungQK_FR = 0;
    static int32_t xungQK_RL = 0;
    static int32_t xungQK_RR = 0;

    // --- Biến lưu vận tốc đã lọc (Dùng Low-pass filter) ---
    // Hệ số lọc alpha (0.0 đến 1.0). 
    // Alpha nhỏ -> Lọc mượt hơn nhưng trễ nhiều hơn. (Thử nghiệm với 0.3 - 0.7)
    // const float ALPHA = 0.5f; 
    static float filtered_delta_FL = 0.0f;
    static float filtered_delta_FR = 0.0f;
    static float filtered_delta_RL = 0.0f;
    static float filtered_delta_RR = 0.0f;

    // --- 2. Đọc giá trị Timer hiện tại ---
    int32_t xungHT_FL = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    int32_t xungHT_FR = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t xungHT_RR = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t xungHT_RL = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    // --- 3. Tính chênh lệch xung (Delta) ---
    int32_t raw_delta_FL = xungHT_FL - xungQK_FL;
    int32_t raw_delta_FR = xungHT_FR - xungQK_FR;
    int32_t raw_delta_RL = xungHT_RL - xungQK_RL;
    int32_t raw_delta_RR = xungHT_RR - xungQK_RR;

    // --- 4. Xử lý tràn Timer 16-bit ---
    if (raw_delta_FL > 32768) raw_delta_FL -= 65536; else if (raw_delta_FL < -32768) raw_delta_FL += 65536;
    if (raw_delta_FR > 32768) raw_delta_FR -= 65536; else if (raw_delta_FR < -32768) raw_delta_FR += 65536;
    if (raw_delta_RL > 32768) raw_delta_RL -= 65536; else if (raw_delta_RL < -32768) raw_delta_RL += 65536;
    if (raw_delta_RR > 32768) raw_delta_RR -= 65536; else if (raw_delta_RR < -32768) raw_delta_RR += 65536;

    // --- 5. Gán dấu dựa vào biến điều khiển (Logic của bạn) ---
    // Lưu ý: Dấu 1/0 này phụ thuộc vào cấu hình mạch cầu H của bạn
    if (robot.motor_front_left.dir == 0)  raw_delta_FL = -raw_delta_FL;
    if (robot.motor_front_right.dir == 0) raw_delta_FR = -raw_delta_FR;
    if (robot.motor_rear_left.dir == 0)   raw_delta_RL = -raw_delta_RL;
    if (robot.motor_rear_right.dir == 0)  raw_delta_RR = -raw_delta_RR;

    // --- 6. Áp dụng Low-pass Filter (Chống nhiễu do lật dấu) ---
    // Nếu target = 0, ép vận tốc đo được về 0 nhanh chóng để tránh PID gồng
    if (target_w_fl == 0.0f) filtered_delta_FL = 0.0f;
    // else filtered_delta_FL = (ALPHA * (float)raw_delta_FL) + ((1.0f - ALPHA) * filtered_delta_FL);

    if (target_w_fr == 0.0f) filtered_delta_FR = 0.0f;
    // else filtered_delta_FR = (ALPHA * (float)raw_delta_FR) + ((1.0f - ALPHA) * filtered_delta_FR);

    if (target_w_rl == 0.0f) filtered_delta_RL = 0.0f;
    // else filtered_delta_RL = (ALPHA * (float)raw_delta_RL) + ((1.0f - ALPHA) * filtered_delta_RL);

    if (target_w_rr == 0.0f) filtered_delta_RR = 0.0f;
    // else filtered_delta_RR = (ALPHA * (float)raw_delta_RR) + ((1.0f - ALPHA) * filtered_delta_RR);

    // --- 7. Tính toán PID và Xử lý Target = 0 ---
    float pwm_fl_out = 0.0f, pwm_fr_out = 0.0f, pwm_rl_out = 0.0f, pwm_rr_out = 0.0f;

    // Bánh FL
    if (target_w_fl == 0.0f) {
        pid_motor_fl.Reset();
    } else {
        pwm_fl_out = pid_motor_fl.Compute(target_w_fl, filtered_delta_FL, 0.01f);
    }
    // Bánh FR
    if (target_w_fr == 0.0f) {
        pid_motor_fr.Reset();
    } else {
        pwm_fr_out = pid_motor_fr.Compute(target_w_fr, filtered_delta_FR, 0.01f);
    }
    // Bánh RL
    if (target_w_rl == 0.0f) {
        pid_motor_rl.Reset();
    } else {
        pwm_rl_out = pid_motor_rl.Compute(target_w_rl, filtered_delta_RL, 0.01f);
    }
    // Bánh RR
    if (target_w_rr == 0.0f) {
        pid_motor_rr.Reset();
    } else {
        pwm_rr_out = pid_motor_rr.Compute(target_w_rr, filtered_delta_RR, 0.01f);
    }

    // --- 8. Xuất PWM và cập nhật trạng thái Hướng ---
    uint16_t final_pwm_fl = (pwm_fl_out >= 0) ? (uint16_t)pwm_fl_out : (uint16_t)(-pwm_fl_out);
    uint16_t final_pwm_fr = (pwm_fr_out >= 0) ? (uint16_t)pwm_fr_out : (uint16_t)(-pwm_fr_out);
    uint16_t final_pwm_rl = (pwm_rl_out >= 0) ? (uint16_t)pwm_rl_out : (uint16_t)(-pwm_rl_out);
    uint16_t final_pwm_rr = (pwm_rr_out >= 0) ? (uint16_t)pwm_rr_out : (uint16_t)(-pwm_rr_out);

    // CHỈ CẬP NHẬT HƯỚNG KHI CÓ YÊU CẦU QUAY (tránh lật hướng lung tung khi PWM = 0)
    if (target_w_fl != 0.0f) robot.motor_front_left.dir  = (pwm_fl_out >= 0) ? 1 : 0;
    if (target_w_fr != 0.0f) robot.motor_front_right.dir = (pwm_fr_out >= 0) ? 1 : 0;
    if (target_w_rl != 0.0f) robot.motor_rear_left.dir   = (pwm_rl_out >= 0) ? 1 : 0;
    if (target_w_rr != 0.0f) robot.motor_rear_right.dir  = (pwm_rr_out >= 0) ? 1 : 0;

    MotorCtr_FL.control(final_pwm_fl, static_cast<MotorDir>(robot.motor_front_left.dir));
    MotorCtr_FR.control(final_pwm_fr, static_cast<MotorDir>(robot.motor_front_right.dir));
    MotorCtr_RL.control(final_pwm_rl, static_cast<MotorDir>(robot.motor_rear_left.dir));
    MotorCtr_RR.control(final_pwm_rr, static_cast<MotorDir>(robot.motor_rear_right.dir));

    // --- 9. Cập nhật biến quá khứ ---
    xungQK_FL = xungHT_FL;
    xungQK_FR = xungHT_FR;
    xungQK_RL = xungHT_RL;
    xungQK_RR = xungHT_RR;
}

void tinhTargetWheelSpeeds_c()
{
    tinhTargetWheelSpeeds();
}
