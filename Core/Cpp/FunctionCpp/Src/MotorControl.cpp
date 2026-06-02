#include "MotorControl.hpp"
#include <math.h>
#include "const.hpp"

// Khoi tao dong co robot
MotorControl MotorCtr_FL;
MotorControl MotorCtr_FR;
MotorControl MotorCtr_RL;
MotorControl MotorCtr_RR;

float ALPHA = 0.1;     // He so de tang/giam toc tu tu cho dong co
float delta_ccr = 2.0f; // Khi ccrHT gan bang ccrTL, cho ccrHT = ccrTL

// Ham khoi tao dong co robot
void MotorControl::init(TIM_HandleTypeDef *htimPWM, uint32_t timerChannel, GPIO_TypeDef *dirPort, uint16_t dirPin)
{
    // Lay con tro timer va kenh timer
    this->htimPWM = htimPWM;
    this->timerChannel = timerChannel;
    // Lay dia chi thanh ghi ccr cua kenh timer tuong ung
    switch (this->timerChannel)
    {
    case TIM_CHANNEL_1:
        CCR = &(this->htimPWM->Instance->CCR1);
        break;
    case TIM_CHANNEL_2:
        CCR = &(this->htimPWM->Instance->CCR2);
        break;
    case TIM_CHANNEL_3:
        CCR = &(this->htimPWM->Instance->CCR3);
        break;
    case TIM_CHANNEL_4:
        CCR = &(this->htimPWM->Instance->CCR4);
        break;
    default:
        CCR = nullptr;
        while (1)
            ;
        break;
    }
    // Lấy địa chỉ thanh ghi arr của timer
    this->ARR = this->htimPWM->Instance->ARR;

    // Bat tao xung PWM
    HAL_TIM_PWM_Start(this->htimPWM, this->timerChannel);
    this->stop();

    // Tao chan dieu khien huong
    this->dir.init(dirPort, dirPin);
}

// Ham dieu khien dong co robot
void MotorControl::control(uint16_t speed, MotorDir dir)
{
    // Kiem tra toc do co hop le khong
    if (speed >= 0 && speed <= this->ARR)
    {
        if (dir == MotorDir::Backward)
        {
            this->dir.low(); // Chay lui, reset bit để tat
        }
        else if (dir == MotorDir::Forward)
        {
            this->dir.high(); // Chay tien, set bit để bật
        }

        // Chu ki xung PWM quyet dinh toc do
        // CNT la thanh ghi dem so sanh voi CCR va ARR
        *this->CCR = speed;
    }
    else
    {
        while (1)
            ;
    }
}

// Ham dung dong co
void MotorControl::stop()
{
    *this->CCR = 0;
}

// Khoi tao doi tuong dieu khien robot
RobotDrive_t robot = {
    .state = {
        .isDebugMode = 0,   // 0 = dang chay binh thuong, 1 = dang chay debug
        .isDataNew = 0,     // 0 = chua co du lieu moi de nhan, 1 = da co du lieu moi tu pi
        .isSendDataNew = 1, // 0 = khong co du lieu moi de gui, 1 = co du lieu moi de gui cho pi
        .isControlNew = 0},

    .motor_front_left = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_front_right = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_rear_left = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f},
    .motor_rear_right = {.dir = 0, .ccrHT = 0, .ccrTL = 0, .vanToc = 0.0f}};

// Ham dieu khien dong co tuc thi
void dieuKhienMotor()
{
    MotorCtr_FL.control((uint16_t)fabsf(robot.motor_front_left.ccrTL), static_cast<MotorDir>(robot.motor_front_left.dir));
    MotorCtr_FR.control((uint16_t)fabsf(robot.motor_front_right.ccrTL), static_cast<MotorDir>(robot.motor_front_right.dir));
    MotorCtr_RL.control((uint16_t)fabsf(robot.motor_rear_left.ccrTL), static_cast<MotorDir>(robot.motor_rear_left.dir));
    MotorCtr_RR.control((uint16_t)fabsf(robot.motor_rear_right.ccrTL), static_cast<MotorDir>(robot.motor_rear_right.dir));
}

// Ham dung dong co tu tu
void stopOnDinhCCR()
{
    robot.motor_front_left.ccrTL = 0;
    robot.motor_front_right.ccrTL = 0;
    robot.motor_rear_left.ccrTL = 0;
    robot.motor_rear_right.ccrTL = 0;
}

// Hàm tăng tốc độ từ từ motor
void controlOnDinhCCR()
{
    // Tính muc xung ccr tu tu cho tung banh
    // Bên trái trước
    robot.motor_front_left.ccrHT += ALPHA * (robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT);
    if (fabsf(robot.motor_front_left.ccrTL - robot.motor_front_left.ccrHT) < delta_ccr)
    {
        robot.motor_front_left.ccrHT = robot.motor_front_left.ccrTL;
    }
    // Bên trái sau
    robot.motor_rear_left.ccrHT += ALPHA * (robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT);
    if (fabsf(robot.motor_rear_left.ccrTL - robot.motor_rear_left.ccrHT) < delta_ccr)
    {
        robot.motor_rear_left.ccrHT = robot.motor_rear_left.ccrTL;
    }

    // Bên phải trước
    robot.motor_front_right.ccrHT += ALPHA * (robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT);
    if (fabsf(robot.motor_front_right.ccrTL - robot.motor_front_right.ccrHT) < delta_ccr)
    {
        robot.motor_front_right.ccrHT = robot.motor_front_right.ccrTL;
    }
    // Bên phải sau
    robot.motor_rear_right.ccrHT += ALPHA * (robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT);
    if (fabsf(robot.motor_rear_right.ccrTL - robot.motor_rear_right.ccrHT) < delta_ccr)
    {
        robot.motor_rear_right.ccrHT = robot.motor_rear_right.ccrTL;
    }

    // Tinh huong cua tung banh dựa trên dấu của ccrHT hiện tại
    // Ben trai
    robot.motor_front_left.dir = (robot.motor_front_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    robot.motor_rear_left.dir = (robot.motor_rear_left.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    // Ben phai, khong dao dau vi chinh cho cung chieu roi
    robot.motor_front_right.dir = (robot.motor_front_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);
    robot.motor_rear_right.dir = (robot.motor_rear_right.ccrHT >= 0) ? static_cast<uint8_t>(MotorDir::Forward) : static_cast<uint8_t>(MotorDir::Backward);

    // // ĐIỀU KHIỂN MOTOR
    // MotorCtr_FL.control((uint16_t)fabsf(robot.motor_front_left.ccrHT), static_cast<MotorDir>(robot.motor_front_left.dir));
    // MotorCtr_FR.control((uint16_t)fabsf(robot.motor_front_right.ccrHT), static_cast<MotorDir>(robot.motor_front_right.dir));
    // MotorCtr_RL.control((uint16_t)fabsf(robot.motor_rear_left.ccrHT), static_cast<MotorDir>(robot.motor_rear_left.dir));
    // MotorCtr_RR.control((uint16_t)fabsf(robot.motor_rear_right.ccrHT), static_cast<MotorDir>(robot.motor_rear_right.dir));

    // Dieu khien dong co du tren du lieu dau vao
    // target_w_fl = robot.motor_front_left.ccrHT;
    // target_w_fr = robot.motor_front_right.ccrHT;
    // target_w_rl = robot.motor_rear_left.ccrHT;
    // target_w_rr = robot.motor_rear_right.ccrHT;
}