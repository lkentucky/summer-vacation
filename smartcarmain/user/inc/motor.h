#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0                                      //左轮方向
#define MOTORR_DIR A1                                      //右轮方向
#define MOTORL_PWM TIM5_PWM_CH3_A2                         //左轮PWM
#define MOTORR_PWM TIM5_PWM_CH4_A3                         //右轮PWM

// 新板电机直通测试：1=双击K4后两轮固定PWM，0=恢复正常串级方向环和速度PID。
#define MOTOR_PWM_TEST_ENABLE  (0)
// 电机直通测试的固定PWM绝对值；正数表示按当前正转方向运行。
#define MOTOR_PWM_TEST_DUTY    (500)
// 测试模式下不依赖摄像头边线，因此关闭图像丢线停车判定，双击K4仍可停车。
#define MOTOR_PWM_TEST_IGNORE_TRACK_LOST (1)

extern int motor_speedl;  // 左轮速度
extern int motor_speedr;  // 右轮速度
extern int encoder_diffl;  // 左轮编码器差值
extern int encoder_diffr;  // 右轮编码器差值
// 固定PWM测试期间累计的左编码器原始计数；左轮正转按现有换算应为正数。
extern volatile int32 encoder_test_total_l;
// 固定PWM测试期间累计的右编码器原始计数；右轮正转按现有换算应为负数。
extern volatile int32 encoder_test_total_r;
extern float real_speedl;  // 左轮实际速度
extern float real_speedr;  // 右轮实际速度
extern float Kp;  // PID控制器的比例增益
extern float Ki;  // PID控制器的积分增益
extern float Kd;  // PID控制器的微分增益
extern float target_speedl;  // 左轮目标速度
extern float target_speedr;  // 右轮目标速度

extern int base_speed;       // 当前运行速度，0 表示停车
extern int run_base_speed;   // 菜单可调的启动/巡线速度
extern int speed_tier_ratio_1;   // 四档速度相对run_base_speed的百分比，菜单可调
extern int speed_tier_ratio_2;
extern int speed_tier_ratio_3;
extern int speed_tier_ratio_4;
extern int speed_tier_accel_step; // 每图像帧最大升速量，单位cm/s
extern int speed_tier_decel_step; // 每图像帧最大降速量，单位cm/s
extern volatile int speed_tier_current; // 当前速度档：0停车/等待，1~4为速度档
// 蓝牙摇杆遥控：输入范围-100..100，单轮最大速度200cm/s（2m/s）。
#define JOYSTICK_MAX_SPEED_CM_S (200.0f)
#define JOYSTICK_TURN_RATIO     (0.25f)
#define JOYSTICK_TIMEOUT_MS     (300U)
#define JOYSTICK_DEADZONE       (5)
extern volatile uint8 joystick_control_active;
extern volatile int joystick_turn_percent;
extern volatile int joystick_forward_percent;
void motor_joystick_set(int turn_percent, int forward_percent);
void motor_joystick_stop(void);
void motor_auto_start(void);       // 从base_speed=0开始，按spd_up逐图像帧加速
uint8 motor_auto_is_running(void); // 包含尚未产生第一步速度的起步阶段
extern float vision_yaw_kp;           // 视觉外环P系数，单位(deg/s)/pixel
extern float vision_yaw_kq;           // 有效P随误差绝对值增加的斜率
extern float vision_yaw_kp_max;       // 视觉外环最大有效P
extern float vision_error_deadband;   // 视觉横向误差死区，单位pixel
extern float vision_yaw_kd;           // 视觉外环D系数，单位deg/pixel
extern float vision_yaw_kff;          // 远点相对加权偏差的预瞄前馈系数，单位(deg/s)/pixel
extern float yaw_rate_kp;             // 角速度内环P系数，单位(cm/s)/(deg/s)
extern int yaw_rate_limit_dps;      // 视觉外环最大期望角速度，单位deg/s
extern float yaw_rate_feedback_sign;  // 陀螺仪反馈方向/比例，绝对值决定IMU角速度抑制强度
extern volatile float yaw_rate_ref_dps;   // 视觉外环当前期望角速度，单位deg/s
extern volatile float yaw_rate_error_dps; // 角速度内环当前误差，单位deg/s
extern volatile int steering_image_error_display; // image菜单显示的实际加权巡线误差
extern volatile float steering_heading_error_deg_display; // 调速使用的有符号航向角误差，单位deg


void motor_init(void);
void motorl_set_pwm(int lpwm);
void motorr_set_pwm(int rpwm);
void init_encoder(void);
void get_motor_speed(void);
// 每个图像帧更新视觉外环；加权偏差负责主反馈，远点负责预瞄。
void steering_set_image_error(int16 error_weighted, int16 error_near,
                               int16 error_far, float image_dt_s);
int16 steering_get_image_error(void); // 读取最近一帧加权中线偏差，供无线遥测使用。
int16 steering_get_heading_error(void); // 读取调速使用的航向角误差绝对值，单位deg。
// 每10ms执行角速度内环，根据期望角速度和IMU角速度更新左右轮目标速度。
void steering_control_update(void);
void motor_pid_speedcontrol(void);
void motor_pid_reset(void);

#endif  // __MOTOR_H__

