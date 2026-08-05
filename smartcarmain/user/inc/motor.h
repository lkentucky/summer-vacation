#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0                                      //左轮方向
#define MOTORR_DIR A1                                      //右轮方向
#define MOTORL_PWM TIM5_PWM_CH3_A2                         //左轮PWM
#define MOTORR_PWM TIM5_PWM_CH4_A3                         //右轮PWM

// 速度决策总开关：1=启用速度决策，0=禁用并继续使用run_base_speed。
#define SPEED_DECISION_ENABLE (1)
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
extern float vision_yaw_kp;           // 视觉外环P系数，单位(deg/s)/pixel
extern float vision_yaw_kd;           // 视觉外环D系数，单位deg/pixel
extern float yaw_rate_kp;             // 角速度内环P系数，单位(cm/s)/(deg/s)
extern float yaw_rate_limit_dps;      // 视觉外环最大期望角速度，单位deg/s
extern float yaw_rate_feedback_sign;  // 陀螺仪安装方向修正，只使用1或-1
extern volatile float yaw_rate_ref_dps;   // 视觉外环当前期望角速度，单位deg/s
extern volatile float yaw_rate_error_dps; // 角速度内环当前误差，单位deg/s

#if SPEED_DECISION_ENABLE
// 速度状态机：菜单中spd_state显示0=直道，1=弯道，2=摆动抑制。
enum
{
  SPEED_STATE_STRAIGHT = 0,
  SPEED_STATE_CORNER = 1,
  SPEED_STATE_OSCILLATION = 2
};

extern int speed_straight_speed;          // 直道目标速度，单位cm/s
extern int speed_corner_speed;            // 弯道目标速度，单位cm/s
extern float speed_straight_yaw_feedback_sign; // 直道角速度反馈方向/比例
extern float speed_corner_yaw_feedback_sign;   // 弯道角速度反馈方向/比例
extern float speed_straight_vision_kp_square;  // 直道/出弯稳定阶段视觉平方P系数
extern float speed_corner_vision_kp_square;    // 弯道视觉平方P系数
extern float speed_oscillation_gyro_threshold; // 摆动检测的最小有效角速度绝对值，单位deg/s
extern int speed_oscillation_reversal_required; // 窗口内触发摆动状态所需的角速度换向次数
extern int speed_state;                   // 当前状态：0=直道，1=弯道，2=摆动抑制
extern int speed_decision_speed;          // 经过直道确认和加减速限制后的速度指令，单位cm/s
extern float speed_accel_step;            // 每个图像帧允许增加的最大速度，单位cm/s
extern float speed_decel_step;            // 每个图像帧允许减少的最大速度，单位cm/s
extern int speed_straight_confirm_frames; // 弯道状态下连续多少帧满足直道条件才切回直道
extern int speed_corner_confirm_frames;   // 直道状态下连续多少帧满足弯道条件才切入弯道

// 使用最新图像误差更新直道/弯道状态及速度指令，每个图像帧调用一次。
void speed_decision_update(void);
// 清空直道计数，并把状态和速度恢复到弯道安全值。
void speed_decision_reset(void);
#endif

void motor_init(void);
void motorl_set_pwm(int lpwm);
void motorr_set_pwm(int rpwm);
void init_encoder(void);
void get_motor_speed(void);
// 每个图像帧更新视觉外环；image_dt_s为真实图像帧间隔，单位s。
void steering_set_image_error(int16 error_near, int16 error_far, float image_dt_s);
// 每2ms执行角速度内环，根据期望角速度和IMU角速度更新左右轮目标速度。
void steering_control_update(void);
void motor_pid_speedcontrol(void);
void motor_pid_reset(void);

#endif  // __MOTOR_H_
