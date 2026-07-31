#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0                                      //左轮方向
#define MOTORR_DIR A2                                      //右轮方向
#define MOTORL_PWM TIM5_PWM_CH2_A1                         //左轮PWM
#define MOTORR_PWM TIM5_PWM_CH4_A3                         //右轮PWM

// 速度决策总开关：1=启用速度决策，0=禁用并继续使用run_base_speed。
#define SPEED_DECISION_ENABLE (1)

extern int motor_speedl;  // 左轮速度
extern int motor_speedr;  // 右轮速度
extern int encoder_diffl;  // 左轮编码器差值
extern int encoder_diffr;  // 右轮编码器差值
extern float real_speedl;  // 左轮实际速度
extern float real_speedr;  // 右轮实际速度
extern float Kp;  // PID控制器的比例增益
extern float Ki;  // PID控制器的积分增益
extern float Kd;  // PID控制器的微分增益
extern float target_speedl;  // 左轮目标速度
extern float target_speedr;  // 右轮目标速度

extern int base_speed;       // 当前运行速度，0 表示停车
extern int run_base_speed;   // 菜单可调的启动/巡线速度
extern float Kp_steer;     // 方向P系数
extern float Kp_steer_square;     // 平方项P系数
extern float Kd_steer_position;     // 方向D系数
extern float Kd_steer_time;     // 方向D系数

#if SPEED_DECISION_ENABLE
// 速度状态机的两个状态；菜单中spd_state显示0表示直道，1表示弯道。
enum
{
  SPEED_STATE_STRAIGHT = 0,
  SPEED_STATE_CORNER = 1
};

extern int speed_straight_speed;          // 直道目标速度，单位cm/s
extern int speed_corner_speed;            // 弯道目标速度，单位cm/s
extern int speed_state;                   // 当前状态：0=直道，1=弯道
extern int speed_decision_speed;          // 经过直道确认和加减速限制后的速度指令，单位cm/s
extern float speed_accel_step;            // 每个图像帧允许增加的最大速度，单位cm/s
extern float speed_decel_step;            // 每个图像帧允许减少的最大速度，单位cm/s
extern int speed_straight_confirm_frames; // 弯道状态下连续多少帧满足直道条件才切回直道

// 使用最新图像误差和Z轴角速度更新直道/弯道状态及速度指令，每个图像帧调用一次。
void speed_decision_update(void);
// 清空直道计数，并把状态和速度恢复到弯道安全值。
void speed_decision_reset(void);
#endif

void motor_init(void);
void motorl_set_pwm(int lpwm);
void motorr_set_pwm(int rpwm);
void init_encoder(void);
void get_motor_speed(void);
void steering_set_image_error(int16 error_near, int16 error_far);
void steering_control_update(void);
void motor_pid_speedcontrol(void);
void motor_pid_reset(void);

#endif  // __MOTOR_H_
