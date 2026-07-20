#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0                                      //左轮方向
#define MOTORR_DIR A2                                      //右轮方向
#define MOTORL_PWM TIM5_PWM_CH2_A1                         //左轮PWM
#define MOTORR_PWM TIM5_PWM_CH4_A3                         //右轮PWM

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

extern int base_speed;   // 巡线基础速度
extern float Kp_steer;     // 方向P系数
extern float Kd_steer;     // 方向D系数

void motor_init(void);
void motorl_set_pwm(int lpwm);
void motorr_set_pwm(int rpwm);
void init_encoder(void);
void get_motor_speed(void);
void motor_pid_speedcontrol(void);
void motor_pid_reset(void);

#endif  // __MOTOR_H_
