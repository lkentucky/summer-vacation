#include "motor.h"
#include "IMU.h"

#define D 6.5   //轮子直径
#define PPR 1024 //编码器每转脉冲数
#define STEER_MAX_RATIO       (0.85f)
#define STEER_MAX_STEP        (15.0f)
float Kp = 9.36f;
float Ki = 0.5f;
float Kd = 0.01f;

int motor_speedl = 0;
int motor_speedr = 0;   
int encoder_diffl = 0;
int encoder_diffr = 0;
float real_speedl = 0.0f;
float real_speedr = 0.0f;
float target_speedl = 0.0f;  // 左轮目标速度
float target_speedr = 0.0f;  // 右轮目标速度

int base_speed = 0;     // 当前运行速度，0 表示停车
int run_base_speed = 350; // 菜单可调的启动/巡线速度，K4 启动时赋给 base_speed
float Kp_steer = 3.12f;     // 方向P系数
float Kd_steer_position = 0.0f;     // 方向D系数
float Kd_steer_time = 0.01f;     // 方向D系数

static volatile int16 steer_error_near = 0;  // 图像处理输出的近处偏差，由主循环按帧更新
static volatile int16 steer_error_far = 0;   // 图像处理输出的远处偏差，由主循环按帧更新
static int16 steer_last_error = 0;
static float steer_last_output = 0.0f;

static float motor_limit_float(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

void steering_set_image_error(int16 error_near, int16 error_far)
{
  steer_error_near = error_near;
  steer_error_far = error_far;
}

// 轻量级转向环：只使用上一帧图像误差 + 当前IMU滤波值，不做图像处理、不读IMU硬件。
// 由TIM6每2ms调用，使左右轮目标速度刷新频率高于图像处理频率。
void steering_control_update(void)
{
  if (base_speed > 0) {
    int16 error_near = steer_error_near;
    int16 error_far = steer_error_far;
    int16 preview_error = error_far - error_near;
    int16 d_error = error_near - steer_last_error;

    float steering = Kp_steer * error_far
                   + Kd_steer_position * preview_error
                   + Kd_steer_time * d_error
                   + imu_get_steer_damping();

    float max_steering = (float)base_speed * STEER_MAX_RATIO;
    steering = motor_limit_float(steering, -max_steering, max_steering);
    steering = motor_limit_float(steering,
                                 steer_last_output - STEER_MAX_STEP,
                                 steer_last_output + STEER_MAX_STEP);

    steer_last_error = error_near;
    steer_last_output = steering;

    target_speedl = base_speed + steering;
    target_speedr = base_speed - steering;
  } else {
    steer_last_error = 0;
    steer_last_output = 0.0f;
    target_speedl = 0.0f;
    target_speedr = 0.0f;
  }
}

void motor_init(void) 
{
  // 初始化电机控制引脚
  gpio_init(MOTORL_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(MOTORR_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);

  // 初始化定时器用于PWM输出
  pwm_init(MOTORL_PWM, 5000, 0);
  pwm_init(MOTORR_PWM, 5000, 0);
}


void motorl_set_pwm(int lpwm) 
{
  // 设置电机速度，pwm范围为-10000到10000
  if (lpwm > 10000) lpwm = 10000;
  if (lpwm < -10000) lpwm = -10000;

  if (lpwm >= 0) 
  {
    gpio_high(MOTORL_DIR); // 设置左轮正转
    pwm_set_duty(MOTORL_PWM, lpwm);
  } 
  else 
  {
    gpio_low(MOTORL_DIR); // 设置左轮反转
    pwm_set_duty(MOTORL_PWM, -lpwm);
  }
}

void motorr_set_pwm(int rpwm) 
{
  // 设置电机速度，pwm范围为-10000到10000
  if (rpwm > 10000) rpwm = 10000;
  if (rpwm < -10000) rpwm = -10000;

  if (rpwm >= 0) 
  {
    gpio_high(MOTORR_DIR); // 设置右轮正转
    pwm_set_duty(MOTORR_PWM, rpwm);
  } 
  else 
  {
    gpio_low(MOTORR_DIR); // 设置右轮反转
    pwm_set_duty(MOTORR_PWM, -rpwm);
  }
}

void init_encoder(void) 
{
  // 初始化编码器引脚
  encoder_quad_init(TIM3_ENCODER,TIM3_ENCODER_CH1_B4,TIM3_ENCODER_CH2_B5);
  encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_B6,TIM4_ENCODER_CH2_B7);
}


void get_motor_speed(void) 
{
  // 获取编码器计数值
  
  real_speedl = (float)motor_speedl / PPR * (D * PI); // 单位cm/s
  real_speedr = -(float)motor_speedr / PPR * (D * PI); // 单位cm/s
}


static float errl_k1, errl_k2;
static float errr_k1, errr_k2;
static float control_effortl, control_effortr;

void motor_pid_reset(void)
{
    errl_k1 = errl_k2 = 0;
    errr_k1 = errr_k2 = 0;
    control_effortl = control_effortr = 0;
    steer_last_error = 0;
    steer_last_output = 0.0f;
}

//pid闭环控制电机转速
void motor_pid_speedcontrol(void)
{

    float errl = target_speedl - real_speedl;
    float errr = target_speedr - real_speedr;

    float deltal = Kp * (errl - errl_k1)
                + Ki * errl
                + Kd * (errl - 2*errl_k1 + errl_k2);

    float deltar = Kp * (errr - errr_k1)
                + Ki * errr
                + Kd * (errr - 2*errr_k1 + errr_k2);

    errl_k2 = errl_k1;
    errl_k1 = errl;
    control_effortl += deltal;
    if (control_effortl >  5000) control_effortl =  5000;   // 积分限幅
    if (control_effortl < -5000) control_effortl = -5000;

    errr_k2 = errr_k1;
    errr_k1 = errr;
    control_effortr += deltar;
    if (control_effortr >  5000) control_effortr =  5000;
    if (control_effortr < -5000) control_effortr = -5000;

    motorl_set_pwm((int)control_effortl);
    motorr_set_pwm((int)control_effortr);
}
