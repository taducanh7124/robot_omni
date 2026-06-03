#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <stdint.h>
#include "main.h"
#include "PinConfig.hpp"

enum class MotorDir : uint8_t
{
    Backward = 0,
    Forward = 1
};

// Khai bao lop dieu khien dong co
class MotorControl
{
private:
    // tao xung PWM va dieu khien huong di
    TIM_HandleTypeDef *htimPWM; // timer dùng xuất xung pwm
    uint32_t timerChannel;      // kenh cua timer dung de xuat pwm

    PinConfig_Out dir;     // chan dieu khien huong di
    GPIO_TypeDef *dirPort; // Port của chân điều khiển hướng
    uint16_t dirPin;       // Pin của chân điều khiển hướng

    volatile uint32_t ARR;  // Auto-reload register (gia tri toi da cua timer, = 100% duty cycle)
    volatile uint32_t *CCR; // Capture/Compare Register (gia tri de so sanh de xuat xung, = duty cycle hien tai)
public:
    void init(TIM_HandleTypeDef *htim, uint32_t timerChannel, GPIO_TypeDef *dirPort, uint16_t dirPin);
    void control(uint16_t speed, MotorDir dir);
    void stop();
};

// Bo dieu khien pid
typedef struct {
    float kp; // He so ti le k
    float ki; // He so tich phan i
    float kd; // He so dao ham d
    float saiSoCongDon; // Sai so cong don (tich phan)
    float saiSoCu; // Sai so cu (dao ham)
} PID_t;

// Du lieu dieu khien robot
typedef struct
{
    uint8_t dir;
    float ccrHT;      // CCR hiện tại
    float ccrTL;      // CCR tương lai mong muốn đạt được
    float vanToc;     // Van toc hien tai
    float vanTocDich; // Van toc dich mong muon dat duoc
    PID_t pid;        // Bo dieu khien PID tuong ung
} DataControl_t;

// Trang thai cua robot
typedef struct
{
    uint8_t isDebugMode;   // Flag cho biet robot dang o trang thai debug hay khong
    uint8_t isDataNew;     // Flag co du lieu moi nhan tu pi, 0 = chua co du lieu moi de nhan, 1 = da co du lieu moi tu pi
    uint8_t isSendDataNew; // Flag co du lieu moi gui cho pi, 0 = khong co du lieu moi de gui, 1 = co du lieu moi de gui cho pi
    uint8_t isControlNew;  // Flag co du lieu dieu khien moi de cap nhat dong co
} State_t;

// Trạng thái động cơ robot
typedef struct
{
    State_t state;
    DataControl_t motor_front_left;
    DataControl_t motor_front_right;
    DataControl_t motor_rear_left;
    DataControl_t motor_rear_right;
} RobotDrive_t;

// khai bao toan cuc
extern MotorControl MotorCtr_FL;
extern MotorControl MotorCtr_FR;
extern MotorControl MotorCtr_RL;
extern MotorControl MotorCtr_RR;
extern RobotDrive_t robot;

// Khai bao ham dieu khien
void dieuKhienMotor(void);
void controlOnDinhCCR(void);
void tinhPID(DataControl_t *motor, float delta_t);
void dieuKhienMotorPID(float delta_t);
#endif