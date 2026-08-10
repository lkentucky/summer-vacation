#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define MOTORL_DIR A0
#define MOTORR_DIR A1
#define MOTORL_PWM TIM5_PWM_CH3_A2
#define MOTORR_PWM TIM5_PWM_CH4_A3

#define SPEED_DECISION_ENABLE (1)
#define MOTOR_PWM_TEST_ENABLE  (0)
#define MOTOR_PWM_TEST_DUTY    (500)
#define MOTOR_PWM_TEST_IGNORE_TRACK_LOST (1)

#define JOYSTICK_MAX_SPEED_CM_S (200.0f)
#define JOYSTICK_TURN_RATIO     (0.25f)
#define JOYSTICK_TIMEOUT_MS     (300U)
#define JOYSTICK_DEADZONE       (5)

enum
{
    SPEED_STATE_STRAIGHT = 0,
    SPEED_STATE_CORNER = 1
};

extern int motor_speedl;
extern int motor_speedr;
extern int encoder_diffl;
extern int encoder_diffr;
extern volatile int32 encoder_test_total_l;
extern volatile int32 encoder_test_total_r;
extern float real_speedl;
extern float real_speedr;
extern float Kp;
extern float Ki;
extern float Kd;
extern float target_speedl;
extern float target_speedr;
extern int base_speed;
extern int run_base_speed;

extern volatile uint8 joystick_control_active;
extern volatile int joystick_turn_percent;
extern volatile int joystick_forward_percent;

/* 参考库PPDD方向环：中线P + 帧间D - 陀螺仪阻尼。平方项按要求固定为0。 */
extern float steer_ppdd_kp;
extern float steer_ppdd_kd;
extern float steer_ppdd_gyro_k;
extern volatile float steer_ppdd_output;

extern int speed_straight_speed;
extern int speed_mid_fast_speed;
extern int speed_mid_slow_speed;
extern int speed_corner_speed;
extern int speed_state;
extern int speed_level;
extern int speed_decision_speed;
extern int speed_launch_frames;
extern int speed_launch_count;

void motor_init(void);
void motorl_set_pwm(int lpwm);
void motorr_set_pwm(int rpwm);
void init_encoder(void);
void get_motor_speed(void);
void motor_pid_speedcontrol(void);
void motor_pid_reset(void);
void motor_joystick_set(int turn_percent, int forward_percent);
void motor_joystick_stop(void);

/* steering_value为近处90行中线加权偏差，单位pixel，右偏为正。 */
void steering_set_image_error(float steering_value);
int16 steering_get_image_error(void);
int16 steering_get_far_error(void);
int16 steering_get_speed_error(void);
void steering_control_update(void);
void speed_decision_update(void);
void speed_decision_reset(void);

#endif
