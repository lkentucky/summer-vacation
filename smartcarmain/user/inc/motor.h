#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0                                      //左轮方向
#define MOTORR_DIR A2                                      //右轮方向
#define MOTORL_PWM TIM5_PWM_CH2_A1                         //左轮PWM
#define MOTORR_PWM TIM5_PWM_CH4_A3                         //右轮PWM

// 速度决策草案总开关：当前保持0，不参与编译，也不会改变现有车辆行为。
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
extern int speed_straight_speed;          // 确认直道后的最高速度
extern int speed_corner_speed;            // 普通弯速度，默认保持当前安全速度
extern int speed_extreme_corner_speed;    // 急弯最低速度
extern float speed_curve_score;           // 弯道强度，范围0~1
extern int speed_decision_speed;          // 平滑后的最终速度指令
extern float speed_accel_step;            // 每帧最大加速量
extern float speed_decel_step;            // 每帧最大减速量
extern int speed_straight_confirm_frames; // 连续多少帧为直道才允许加速

void speed_decision_update(void);
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
