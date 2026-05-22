#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <stdint.h>
#include "main.h"
#include "tim.h"

// ========================================================
// 1. VÙNG NÀY CHỈ DÀNH CHO TRÌNH BIÊN DỊCH C++
// Bất cứ gì liên quan đến class, object phải nằm trong này
// ========================================================
#ifdef __cplusplus

#include "MotorControl.hpp" // Chuyển include này vào đây vì nó chứa class

class PIDController {
private:
    float kp, ki, kd;
    float out_min, out_max, integral_limit;
    float integral_sum, prev_error;

public:
    PIDController(float p, float i, float d, float limit_max, float limit_min) {
        kp = p; ki = i; kd = d;
        out_max = limit_max; out_min = limit_min;
        integral_limit = limit_max; 
        Reset();
    }
    void Reset() { integral_sum = 0.0f; prev_error = 0.0f; }
    float Compute(float setpoint, float measured_value, float dt) {
        float error = setpoint - measured_value;
        float P = kp * error;
        integral_sum += error * dt;
        if (integral_sum > integral_limit) integral_sum = integral_limit;
        else if (integral_sum < -integral_limit) integral_sum = -integral_limit;
        float I = ki * integral_sum;
        float D = kd * ((error - prev_error) / dt);
        prev_error = error;
        float output = P + I + D;
        if (output > out_max) output = out_max;
        else if (output < out_min) output = out_min;
        return output;
    }
};

void tinhTargetWheelSpeeds();

extern PIDController pid_motor_fl; // Bánh trước trái
extern PIDController pid_motor_fr; // Bánh trước phải
extern PIDController pid_motor_rl; // Bánh sau trái
extern PIDController pid_motor_rr; // Bánh sau phải

extern volatile float target_w_fl;
extern volatile float target_w_fr;
extern volatile float target_w_rl;
extern volatile float target_w_rr;

#endif // __cplusplus


// ========================================================
// 2. VÙNG DÙNG CHUNG CHO CẢ C VÀ C++ (ĐỂ MAIN.C GỌI)
// ========================================================
#ifdef __cplusplus
extern "C" {
#endif

// Các hàm này main.c sẽ nhìn thấy và hiểu được
void tinhTargetWheelSpeeds_c(void);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_HPP