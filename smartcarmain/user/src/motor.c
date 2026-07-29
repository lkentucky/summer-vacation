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
int run_base_speed = 230; //250// 菜单可调的启动/巡线速度，K4 启动时赋给 base_speed，
float Kp_steer = 4.83f;     //4.93// 方向P系数
float Kp_steer_square = 0.23f;     // 平方项P系数
float Kd_steer_position = 0.0f;    //0.09 // 方向D系数
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

#if SPEED_DECISION_ENABLE
/*
 * 速度决策草案（当前由SPEED_DECISION_ENABLE=0禁用）
 *
 * 设计目标：
 * 1. 远近点差值提前识别入弯；
 * 2. 中线偏差大时禁止高速；
 * 3. IMU角速度确认车辆仍处于弯道； 
 * 4. 入弯快速减速，出弯连续确认后缓慢加速。
 *
 * 调试顺序：
 * 先令straight/corner/extreme全部等于当前安全速度，确认curve_score合理，
 * 然后每次只提高straight_speed，不要先提高corner_speed。
 */

#define SPEED_ERROR_STRAIGHT_PX       (4.0f)
#define SPEED_ERROR_FULL_CURVE_PX     (28.0f)
#define SPEED_PREVIEW_STRAIGHT_PX     (3.0f)
#define SPEED_PREVIEW_FULL_CURVE_PX   (20.0f)
#define SPEED_GYRO_STRAIGHT_DPS       (15.0f)
#define SPEED_GYRO_FULL_CURVE_DPS     (100.0f)
#define SPEED_STRAIGHT_SCORE_MAX      (0.18f)
#define SPEED_NORMAL_CORNER_SCORE     (0.70f)

int speed_straight_speed = 300;
int speed_corner_speed = 230;
int speed_extreme_corner_speed = 200;
float speed_curve_score = 1.0f;
int speed_decision_speed = 230;
float speed_accel_step = 4.5f;
float speed_decel_step = 12.0f;
int speed_straight_confirm_frames = 2;

static int speed_straight_frame_count = 0;
static float speed_command = 230.0f;

static float speed_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float speed_max_float(float value_a, float value_b)
{
  return (value_a > value_b) ? value_a : value_b;
}

static float speed_normalize_score(float value, float straight_value, float full_curve_value)
{
  if (value <= straight_value) return 0.0f;
  if (value >= full_curve_value) return 1.0f;
  if (full_curve_value <= straight_value) return 1.0f;
  return (value - straight_value) / (full_curve_value - straight_value);
}

void speed_decision_reset(void)
{
  int reset_speed = speed_corner_speed;
  if (reset_speed < 0) reset_speed = 0;

  speed_straight_frame_count = 0;
  speed_curve_score = 1.0f;
  speed_command = (float)reset_speed;
  speed_decision_speed = reset_speed;
}

void speed_decision_update(void)
{
  int16 error_near;
  int16 error_far;
  float line_error;
  float preview_error;
  float gyro_z;
  float line_score;
  float preview_score;
  float gyro_score;
  float target_speed;
  int straight_speed;
  int corner_speed;
  int extreme_speed;

  if (base_speed <= 0)
  {
    speed_decision_reset();
    return;
  }

  error_near = steer_error_near;
  error_far = steer_error_far;
  line_error = speed_max_float(speed_abs_float((float)error_near),
                               speed_abs_float((float)error_far));
  preview_error = speed_abs_float((float)(error_far - error_near));
  gyro_z = speed_abs_float(imu_gyro_z_dps_filter);

  line_score = speed_normalize_score(line_error,
                                     SPEED_ERROR_STRAIGHT_PX,
                                     SPEED_ERROR_FULL_CURVE_PX);
  preview_score = speed_normalize_score(preview_error,
                                        SPEED_PREVIEW_STRAIGHT_PX,
                                        SPEED_PREVIEW_FULL_CURVE_PX);
  gyro_score = speed_normalize_score(gyro_z,
                                     SPEED_GYRO_STRAIGHT_DPS,
                                     SPEED_GYRO_FULL_CURVE_DPS);

  speed_curve_score = speed_max_float(line_score,
                                      speed_max_float(preview_score, gyro_score));
  speed_curve_score = motor_limit_float(speed_curve_score, 0.0f, 1.0f);

  straight_speed = speed_straight_speed;
  corner_speed = speed_corner_speed;
  extreme_speed = speed_extreme_corner_speed;
  if (straight_speed < 0) straight_speed = 0;
  if (corner_speed > straight_speed) corner_speed = straight_speed;
  if (corner_speed < 0) corner_speed = 0;
  if (extreme_speed > corner_speed) extreme_speed = corner_speed;
  if (extreme_speed < 0) extreme_speed = 0;

  if (speed_curve_score <= SPEED_NORMAL_CORNER_SCORE)
  {
    float ratio = speed_curve_score / SPEED_NORMAL_CORNER_SCORE;
    target_speed = (float)straight_speed +
                   ((float)corner_speed - (float)straight_speed) * ratio;
  }
  else
  {
    float ratio = (speed_curve_score - SPEED_NORMAL_CORNER_SCORE) /
                  (1.0f - SPEED_NORMAL_CORNER_SCORE);
    target_speed = (float)corner_speed +
                   ((float)extreme_speed - (float)corner_speed) * ratio;
  }

  if (speed_curve_score <= SPEED_STRAIGHT_SCORE_MAX)
  {
    if (speed_straight_frame_count < speed_straight_confirm_frames)
      speed_straight_frame_count++;
  }
  else
  {
    speed_straight_frame_count = 0;
  }

  // 没有连续确认直道时，速度不得高于已验证安全的弯道速度。
  if (speed_straight_frame_count < speed_straight_confirm_frames &&
      target_speed > (float)corner_speed)
  {
    target_speed = (float)corner_speed;
  }

  // 快减慢加：入弯优先保证安全，出弯再逐步释放直道速度。
  if (target_speed < speed_command)
  {
    speed_command -= speed_decel_step;
    if (speed_command < target_speed) speed_command = target_speed;
  }
  else if (target_speed > speed_command)
  {
    speed_command += speed_accel_step;
    if (speed_command > target_speed) speed_command = target_speed;
  }

  speed_decision_speed = (int)(speed_command + 0.5f);
}
#endif

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

    float steering = Kp_steer * error_near
                   + Kp_steer_square * error_near * float_abs(error_near)
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
  pwm_init(MOTORL_PWM, 10000, 0);
  pwm_init(MOTORR_PWM, 10000, 0);
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
  
  real_speedl = (float)motor_speedl / PPR /68*30* (D * PI); // 单位cm/s
  real_speedr = -(float)motor_speedr / PPR /68*30* (D * PI); // 单位cm/s
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
#if SPEED_DECISION_ENABLE
    speed_decision_reset();
#endif
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
