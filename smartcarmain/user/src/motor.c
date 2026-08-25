#include "motor.h"
#include "IMU.h"
#include "isr.h"
#include <math.h>

#define D 6.5   //轮子直径
#define PPR 1024 //编码器每转脉冲数
#define STEER_MAX_RATIO       (1.20f)   // 急弯允许内轮轻微反转，提高最大横摆能力。
#define STEER_ATTACK_STEP     (60.0f)   // 每2ms转向加深时允许的最大差速变化，单位cm/s。
#define STEER_RELEASE_STEP    (20.0f)   // 每2ms回正或换向时允许的最大差速变化，单位cm/s。
#define VISION_D_FILTER_HZ      (6.0f)   // 视觉误差微分低通截止频率，单位Hz。
#define SPEED_TIER_ANGLE_1_DEG   (7.0f)  // 等效于远近点横差约3px。
#define SPEED_TIER_ANGLE_2_DEG  (18.0f)  // 等效于远近点横差约8px。
#define SPEED_TIER_ANGLE_3_DEG  (31.0f)  // 等效于远近点横差约15px。
#define SPEED_HEADING_ROW_DISTANCE_PX (25.0f) // 近点62.5行与远点37.5行的纵向间距。
#define RAD_TO_DEG              (57.29577951f)
#define VISION_DEFAULT_DT_S     (0.020f) // 首帧或异常帧间隔时使用的默认周期，单位s。
#define VISION_MIN_DT_S         (0.005f) // 接受的最小图像周期，防止微分被异常小dt放大。
#define VISION_MAX_DT_S         (0.120f) // 接受的最大图像周期，超过后按默认周期处理。
#define VISION_STALE_TIMEOUT_MS (120U)  // 图像超过该时间未更新时，期望角速度自动归零。
float Kp = 9.36f;
float Ki = 0.5f;
float Kd = 0.01f;

int motor_speedl = 0;
int motor_speedr = 0;   
int encoder_diffl = 0;
int encoder_diffr = 0;
// 本次固定PWM运行期间的左编码器原始累计计数，下一次启动时清零。
volatile int32 encoder_test_total_l = 0;
// 本次固定PWM运行期间的右编码器原始累计计数，下一次启动时清零。
volatile int32 encoder_test_total_r = 0;
float real_speedl = 0.0f;
float real_speedr = 0.0f;
float target_speedl = 0.0f;  // 左轮目标速度
float target_speedr = 0.0f;  // 右轮目标速度

int base_speed = 0;     // 当前运行速度，0 表示停车
int run_base_speed = 315; //250// 菜单可调的启动/巡线速度，K4 启动时赋给 base_speed，
int speed_tier_ratio_1 = 100; // |head_err|<=7deg时相对run_base_speed的百分比。
int speed_tier_ratio_2 = 92;  // 7<|head_err|<=18deg时的速度百分比。
int speed_tier_ratio_3 = 82;  // 18<|head_err|<=31deg时的速度百分比。
int speed_tier_ratio_4 = 74;  // |head_err|>31deg时的速度百分比。
int speed_tier_accel_step = 20; // 每个图像帧允许的最大升速量，单位cm/s。
int speed_tier_decel_step = 70; // 每个图像帧允许的最大降速量，单位cm/s。
volatile int speed_tier_current = 0; // 当前速度档：停车/等待为0，运行时为1~4。
volatile uint8 joystick_control_active = 0;
static volatile uint8 auto_run_requested = 0; // 已请求自动巡线；起步阶段base_speed仍可为0。
volatile int joystick_turn_percent = 0;
volatile int joystick_forward_percent = 0;
static volatile float joystick_target_speedl = 0.0f;
static volatile float joystick_target_speedr = 0.0f;
static volatile uint32 joystick_last_packet_tick = 0;
// 视觉外环变比例P：小误差低增益抑制直道摇摆，大误差自动提高增益增强过弯。
float vision_yaw_kp = 3.0f;        // 最小有效P，单位(deg/s)/pixel。
float vision_yaw_kq = 0.14f;       // P随误差绝对值增加的斜率。
float vision_yaw_kp_max = 7.70f;    // 最大有效P，防止大误差时增益无限增加。
float vision_error_deadband = 1.0f;// 横向误差死区，单位pixel。
// 视觉外环D系数：误差变化速度转换为期望角速度的系数，单位deg/pixel。
float vision_yaw_kd = 0.08f;
// 远点相对加权偏差的预瞄前馈系数，单位(deg/s)/pixel。
float vision_yaw_kff = 0.50f;
// 固定使用的一套角速度内环P系数。
float yaw_rate_kp = 1.53f;//1.53
// 视觉外环允许输出的最大期望角速度绝对值，单位deg/s。
int yaw_rate_limit_dps = 180;
// 固定使用的IMU角速度反馈方向/比例。
float yaw_rate_feedback_sign = -0.5f;
// 视觉外环输出的期望角速度，主循环写入、10ms方向内环读取，单位deg/s。
volatile float yaw_rate_ref_dps = 0.0f;
// 角速度内环当前误差，供菜单观察，单位deg/s。
volatile float yaw_rate_error_dps = 0.0f;

static volatile int16 steer_error_weighted = 0; // 加权中线偏差，不是单独近点，由主循环按帧更新。
static volatile int16 steer_error_near = 0;     // 60~65行近点中线平均偏差，用于计算车身航向。
static volatile int16 steer_error_far = 0;      // 35~40行远点中线平均偏差，由主循环按帧更新。
volatile int steering_image_error_display = 0; // image菜单只读观察值，不参与控制计算。
volatile float steering_heading_error_deg_display = 0.0f; // 调速实际使用的航向角误差。
static int16 vision_last_weighted_error = 0;    // 上一图像帧的加权偏差，用于计算真实视觉微分。
static float vision_error_rate_filter = 0.0f;      // 低通后的视觉误差变化速度，单位pixel/s。
static float vision_preview_error_filter = 0.0f;   // 低通后的“远点-加权偏差”预瞄增量，单位pixel。
static bool vision_last_error_valid = false;       // false表示尚无上一帧，首帧不计算微分。
static volatile uint32 vision_last_image_tick = 0; // 最近一次视觉外环更新时刻，单位2ms系统tick。
static float steer_last_output = 0.0f;             // 上一次左右轮差速指令，用于限制每2ms变化量。

static float motor_limit_float(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

static int speed_limit_percent(int percent)
{
  if (percent < 0) return 0;
  if (percent > 100) return 100;
  return percent;
}

// 直接按本帧航向角误差选择速度，不受车辆在直道中的横向位置影响。
static void steering_update_tiered_speed(float heading_error_deg)
{
  float error_abs;
  int target_speed;
  int target_ratio;
  int accel_step;
  int decel_step;

  if (!auto_run_requested || joystick_control_active) return;
  if (run_base_speed <= 0) {
    base_speed = 0;
    speed_tier_current = 0;
    return;
  }

  error_abs = float_abs(heading_error_deg);
  if (error_abs <= SPEED_TIER_ANGLE_1_DEG) {
    speed_tier_current = 1;
    target_ratio = speed_tier_ratio_1;
  } else if (error_abs <= SPEED_TIER_ANGLE_2_DEG) {
    speed_tier_current = 2;
    target_ratio = speed_tier_ratio_2;
  } else if (error_abs <= SPEED_TIER_ANGLE_3_DEG) {
    speed_tier_current = 3;
    target_ratio = speed_tier_ratio_3;
  } else {
    speed_tier_current = 4;
    target_ratio = speed_tier_ratio_4;
  }
  target_speed = run_base_speed * speed_limit_percent(target_ratio) / 100;
  accel_step = (speed_tier_accel_step > 0) ? speed_tier_accel_step : 0;
  decel_step = (speed_tier_decel_step > 0) ? speed_tier_decel_step : 0;

  if (target_speed < base_speed) {
    base_speed -= decel_step;
    if (base_speed < target_speed) base_speed = target_speed;
  } else if (target_speed > base_speed) {
    base_speed += accel_step;
    if (base_speed > target_speed) base_speed = target_speed;
  }
}


// 每个新图像帧调用一次：加权偏差用于主反馈，远点与加权偏差之差用于预瞄前馈。
// error_weighted是多行加权结果，不是严格几何近点，因此前馈量按预瞄增量而非远近两点斜率处理。
void steering_set_image_error(int16 error_weighted, int16 error_near,
                               int16 error_far, float image_dt_s)
{
  float dt;               // 检查范围后的实际图像周期，单位s。
  float raw_error_rate;   // 本帧横向偏差变化速度，单位pixel/s。
  float raw_preview_error;// 本帧远点相对加权偏差的预瞄增量，单位pixel。
  float filter_tau;       // 视觉微分和预瞄前馈共用的一阶低通时间常数，单位s。
  float filter_alpha;     // 一阶低通中上一结果所占比例。
  float yaw_limit;        // 检查为非负数后的期望角速度限幅，单位deg/s。
  float yaw_ref;          // 本帧视觉反馈与预瞄前馈合成的期望角速度，单位deg/s。
  float image_error;      // 本帧用于主反馈的加权中线偏差，单位pixel。
  float control_error;    // 扣除死区后真正进入P项的横向误差，单位pixel。
  float control_error_abs;// control_error绝对值，单位pixel。
  float error_deadband;   // 检查为非负数后的误差死区，单位pixel。
  float kp_min;           // 检查为非负数后的最小有效P。
  float kp_max;           // 保证不小于kp_min后的最大有效P。
  float kp_slope;         // 检查为非负数后的P增益斜率。
  float kp_effective;     // 本帧根据误差连续计算出的实际P系数。
  float heading_error_deg;// 远近点构成的赛道航向角误差，单位deg。

  steer_error_weighted = error_weighted;
  steer_error_near = error_near;
  steer_error_far = error_far;
  steering_image_error_display = error_weighted;
  heading_error_deg = atan2f((float)(error_far - error_near),
                             SPEED_HEADING_ROW_DISTANCE_PX) * RAD_TO_DEG;
  steering_heading_error_deg_display = heading_error_deg;
  steering_update_tiered_speed(heading_error_deg);
  // 使用真实帧间隔计算微分；首帧和异常间隔不产生微分冲击。
  dt = image_dt_s;
  if (dt < VISION_MIN_DT_S || dt > VISION_MAX_DT_S)
  {
    dt = VISION_DEFAULT_DT_S;
  }

  raw_preview_error = (float)error_far - (float)error_weighted;
  filter_tau = 1.0f / (2.0f * 3.14159265359f * VISION_D_FILTER_HZ);
  filter_alpha = filter_tau / (filter_tau + dt);

  if (vision_last_error_valid)
  {
    raw_error_rate = ((float)error_weighted - (float)vision_last_weighted_error) / dt;
    vision_error_rate_filter = filter_alpha * vision_error_rate_filter +
                               (1.0f - filter_alpha) * raw_error_rate;
    vision_preview_error_filter = filter_alpha * vision_preview_error_filter +
                                  (1.0f - filter_alpha) * raw_preview_error;
  }
  else
  {
    vision_error_rate_filter = 0.0f;
    vision_preview_error_filter = raw_preview_error;
    vision_last_error_valid = true;
  }

  vision_last_weighted_error = error_weighted;
  vision_last_image_tick = g_sys_tick;

  yaw_limit = (yaw_rate_limit_dps >= 0.0f) ?
              yaw_rate_limit_dps : -yaw_rate_limit_dps;
  image_error = (float)error_weighted;
  error_deadband = float_abs(vision_error_deadband);
  control_error_abs = float_abs(image_error);
  if (control_error_abs <= error_deadband) {
    control_error = 0.0f;
    control_error_abs = 0.0f;
  } else {
    control_error_abs -= error_deadband;
    control_error = (image_error < 0.0f) ? -control_error_abs : control_error_abs;
  }

  kp_min = (vision_yaw_kp > 0.0f) ? vision_yaw_kp : 0.0f;
  kp_max = (vision_yaw_kp_max > kp_min) ? vision_yaw_kp_max : kp_min;
  kp_slope = (vision_yaw_kq > 0.0f) ? vision_yaw_kq : 0.0f;
  kp_effective = kp_min + kp_slope * control_error_abs;
  if (kp_effective > kp_max) kp_effective = kp_max;

  yaw_ref = kp_effective * control_error +
            vision_yaw_kff * vision_preview_error_filter +
            vision_yaw_kd * vision_error_rate_filter;
  yaw_rate_ref_dps = motor_limit_float(yaw_ref, -yaw_limit, yaw_limit);
}

int16 steering_get_image_error(void)
{
  return steer_error_weighted;
}

int16 steering_get_heading_error(void)
{
  float heading_error_abs = float_abs(steering_heading_error_deg_display);
  return (int16)(heading_error_abs + 0.5f);
}

void motor_joystick_stop(void)
{
  auto_run_requested = 0;
  speed_tier_current = 0;
  joystick_control_active = 0;
  joystick_turn_percent = 0;
  joystick_forward_percent = 0;
  joystick_target_speedl = 0.0f;
  joystick_target_speedr = 0.0f;
  base_speed = 0;
  target_speedl = 0.0f;
  target_speedr = 0.0f;
  motor_pid_reset();
  motorl_set_pwm(0);
  motorr_set_pwm(0);
}

void motor_auto_start(void)
{
  // 清空上次PID和转向状态，但保持速度为0；下一图像帧按spd_up开始爬升。
  motor_joystick_stop();
  auto_run_requested = 1;
  base_speed = 0;
}

uint8 motor_auto_is_running(void)
{
  return auto_run_requested;
}

void motor_joystick_set(int turn_percent, int forward_percent)
{
  float forward_speed;
  float turn_speed;

  if (turn_percent > 100) turn_percent = 100;
  if (turn_percent < -100) turn_percent = -100;
  if (forward_percent > 100) forward_percent = 100;
  if (forward_percent < -100) forward_percent = -100;
  if (turn_percent >= -JOYSTICK_DEADZONE && turn_percent <= JOYSTICK_DEADZONE) turn_percent = 0;
  if (forward_percent >= -JOYSTICK_DEADZONE && forward_percent <= JOYSTICK_DEADZONE) forward_percent = 0;

  if (!joystick_control_active) motor_pid_reset();
  auto_run_requested = 0;
  speed_tier_current = 0;
  base_speed = 0; // 摇杆模式绕过视觉巡线和自动速度决策。
  joystick_turn_percent = turn_percent;
  joystick_forward_percent = forward_percent;
  forward_speed = (float)forward_percent * (JOYSTICK_MAX_SPEED_CM_S / 100.0f);
  turn_speed = (float)turn_percent * (JOYSTICK_MAX_SPEED_CM_S / 100.0f) * JOYSTICK_TURN_RATIO;
  // 负转向：左轮减速、右轮加速，车辆左转；正转向反之。
  joystick_target_speedl = motor_limit_float(forward_speed + turn_speed,
                                              -JOYSTICK_MAX_SPEED_CM_S,
                                               JOYSTICK_MAX_SPEED_CM_S);
  joystick_target_speedr = motor_limit_float(forward_speed - turn_speed,
                                              -JOYSTICK_MAX_SPEED_CM_S,
                                               JOYSTICK_MAX_SPEED_CM_S);
  joystick_last_packet_tick = g_sys_tick;
  joystick_control_active = 1;
}
// 由TIM6每2ms调用：角速度P内环跟踪视觉外环给出的期望角速度。
// 本函数不处理图像、不读取IMU硬件，只使用主循环已经更新的期望值和陀螺仪滤波值。
void steering_control_update(void)
{
  if (joystick_control_active)
  {
    if ((g_sys_tick - joystick_last_packet_tick) >=
        (JOYSTICK_TIMEOUT_MS + SYS_TICK_MS - 1U) / SYS_TICK_MS)
    {
      motor_joystick_stop();
      return;
    }
    yaw_rate_ref_dps = 0.0f;
    yaw_rate_error_dps = 0.0f;
    steer_last_output = 0.0f;
    target_speedl = joystick_target_speedl;
    target_speedr = joystick_target_speedr;
    return;
  }

  if (base_speed > 0) {
    float yaw_ref = yaw_rate_ref_dps; // 本次内环使用的期望角速度，单位deg/s。
    float yaw_rate_measured;          // 修正安装方向后的实际角速度，单位deg/s。
    float steering;                   // 角速度P环输出的左右轮速度差修正，单位cm/s。
    float max_steering;               // 当前基础速度允许的最大差速修正，单位cm/s。
    float steering_step;              // 本次根据转向加深或释放选择的变化上限。
    bool same_direction;              // 目标差速与当前差速方向是否一致。
    uint32 image_age_ms;              // 距离最近一次视觉更新的时间，单位ms。

    image_age_ms = (g_sys_tick - vision_last_image_tick) * SYS_TICK_MS;
    if (image_age_ms > VISION_STALE_TIMEOUT_MS)
    {
      yaw_ref = 0.0f;
      yaw_rate_ref_dps = 0.0f;
    }

    // yaw_rate_feedback_sign用于统一陀螺仪正方向和左右轮差速正方向。
    yaw_rate_measured = yaw_rate_feedback_sign * imu_gyro_z_dps_filter;
    yaw_rate_error_dps = yaw_ref - yaw_rate_measured;
    steering = yaw_rate_kp * yaw_rate_error_dps;

    max_steering = (float)base_speed * STEER_MAX_RATIO;
    steering = motor_limit_float(steering, -max_steering, max_steering);
    same_direction = (steering == 0.0f || steer_last_output == 0.0f ||
                      steering * steer_last_output > 0.0f);
    steering_step = (same_direction &&
                     float_abs(steering) > float_abs(steer_last_output)) ?
                    STEER_ATTACK_STEP : STEER_RELEASE_STEP;
    steering = motor_limit_float(steering,
                                 steer_last_output - steering_step,
                                 steer_last_output + steering_step);

    steer_last_output = steering;

    target_speedl = base_speed + steering;
    target_speedr = base_speed - steering;
  } else {
    yaw_rate_ref_dps = 0.0f;
    yaw_rate_error_dps = 0.0f;
    vision_last_weighted_error = 0;
    vision_error_rate_filter = 0.0f;
    vision_preview_error_filter = 0.0f;
    vision_last_error_valid = false;
    vision_last_image_tick = g_sys_tick;
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
    gpio_low(MOTORR_DIR); // 设置右轮正转
    pwm_set_duty(MOTORR_PWM, rpwm);
  } 
  else 
  {
    gpio_high(MOTORR_DIR); // 设置右轮反转
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
  
  real_speedl = -(float)motor_speedl / PPR /68*30* (D * PI); // 单位cm/s
  real_speedr = (float)motor_speedr / PPR /68*30* (D * PI); // 单位cm/s
}


static float errl_k1, errl_k2;
static float errr_k1, errr_k2;
static float control_effortl, control_effortr;

void motor_pid_reset(void)
{
    errl_k1 = errl_k2 = 0;
    errr_k1 = errr_k2 = 0;
    control_effortl = control_effortr = 0;
    steer_error_weighted = 0;
    steer_error_near = 0;
    steer_error_far = 0;
    steering_image_error_display = 0;
    steering_heading_error_deg_display = 0.0f;
    vision_last_weighted_error = 0;
    vision_error_rate_filter = 0.0f;
    vision_preview_error_filter = 0.0f;
    vision_last_error_valid = false;
    vision_last_image_tick = g_sys_tick;
    yaw_rate_ref_dps = 0.0f;
    yaw_rate_error_dps = 0.0f;
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
    if (control_effortl >  5000) control_effortl =  5000;   // 限幅阈值和钳位值都设为5000。
    if (control_effortl < -5000) control_effortl = -5000;

    errr_k2 = errr_k1;
    errr_k1 = errr;
    control_effortr += deltar;
    if (control_effortr >  5000) control_effortr =  5000;
    if (control_effortr < -5000) control_effortr = -5000;

    motorl_set_pwm((int)control_effortl);
    motorr_set_pwm((int)control_effortr);
}
