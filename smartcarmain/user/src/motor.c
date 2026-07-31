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
float Kp_steer = 5.44f;     //4.93// 方向P系数
float Kp_steer_square = 0.30f;     // 平方项P系数
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
 * 二状态速度决策（SPEED_DECISION_ENABLE=1启用，=0时不参与编译）
 *
 * STRAIGHT直道状态：
 *   图像中线偏差连续达到入弯阈值，才切换到CORNER。
 * CORNER弯道状态：
 *   图像中线偏差低于出弯阈值，并连续满足指定帧数，才切回STRAIGHT。
 * 入弯阈值大于出弯阈值形成滞回，避免状态在临界值附近反复跳变。
 */

// 直道状态下，最大中线偏差达到该值就判定入弯，单位：像素。
#define SPEED_ENTER_LINE_PX           (15.0f)
// 弯道状态下，最大中线偏差必须低于该值才可能判定出弯，单位：像素。
#define SPEED_EXIT_LINE_PX            (10.0f)

// 弯道降低陀螺仪方向抑制，避免影响转弯响应。
#define SPEED_CORNER_KGYRO_STEER       (1.53f)

// 直道状态的目标速度，单位：cm/s。
int speed_straight_speed = 268;
// 弯道状态的目标速度，单位：cm/s；应设置为实车已验证的安全速度。
int speed_corner_speed = 225;
// 直道方向陀螺仪抑制系数，可在巡线菜单的Kgyro_str中调s整。
float speed_straight_kgyro_steer = 2.0f;
// 当前速度状态；0是直道，1是弯道，复位时默认按更安全的弯道处理。
int speed_state = SPEED_STATE_CORNER;
// 最终提供给base_speed的整数速度指令，单位：cm/s。
int speed_decision_speed = 230;
// 每处理一个图像帧，速度最多增加多少，单位：cm/s/帧。
float speed_accel_step = 4.0f;
// 每处理一个图像帧，速度最多降低多少，单位：cm/s/帧。
float speed_decel_step = 12.0f;
// 在弯道状态下，连续满足多少帧出弯条件后才切换到直道。
int speed_straight_confirm_frames = 4;
// 在直道状态下，连续满足多少帧入弯条件后才切换到弯道。
int speed_corner_confirm_frames = 2;

// 弯道状态下已经连续满足出弯条件的帧数，仅在本文件内部使用。
static int speed_straight_frame_count = 0;
// 直道状态下已经连续满足入弯条件的帧数，仅在本文件内部使用。
static int speed_corner_frame_count = 0;
// 带加减速斜率限制的浮点速度指令，保留小数以避免每帧取整误差，单位：cm/s。
static float speed_command = 230.0f;

// 返回浮点数绝对值，使左右弯使用同一组判断阈值。
static float speed_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

// 停车或控制器复位时，清除直道确认并恢复到弯道安全状态。
void speed_decision_reset(void)
{
  int reset_speed = speed_corner_speed; // 本次使用的复位速度，单位：cm/s。
  if (reset_speed < 0) reset_speed = 0;

  speed_state = SPEED_STATE_CORNER;       // 起步先按弯道处理，防止未知路况直接高速。
  speed_straight_frame_count = 0;         // 清除之前累计的直道帧。
  speed_corner_frame_count = 0;           // 清除之前累计的弯道帧。
  Kgyro_steer = SPEED_CORNER_KGYRO_STEER; // 停车和起步阶段使用弯道抑制系数。
  speed_command = (float)reset_speed;     // 浮点指令回到弯道安全速度。
  speed_decision_speed = reset_speed;     // 对外整数指令同步复位。
}

// 每个新图像帧调用一次：更新状态机，再用快减慢加生成最终速度指令。
void speed_decision_update(void)
{
  int16 error_near;       // 近处中线相对图像中心的有符号偏差，单位：像素。
  int16 error_far;        // 远处中线相对图像中心的有符号偏差，单位：像素。
  float line_error;       // 远近偏差绝对值中的较大者，单位：像素。
  float target_speed;     // 当前状态对应的目标速度，单位：cm/s。
  float accel_step;       // 检查为非负数后的本帧加速步长，单位：cm/s。
  float decel_step;       // 检查为非负数后的本帧减速步长，单位：cm/s。
  int straight_speed;     // 检查为非负数后的直道速度，单位：cm/s。
  int corner_speed;       // 检查范围后的弯道速度，单位：cm/s。

  // 停车时不累计状态，下一次起步仍从弯道安全速度开始。
  if (base_speed <= 0)
  {
    speed_decision_reset();
    return;
  }

  // 读取本帧远近图像偏差，使用绝对值较大的一项判断弯道。
  error_near = steer_error_near;
  error_far = steer_error_far;
  line_error = speed_abs_float((float)error_near);
  if (speed_abs_float((float)error_far) > line_error)
    line_error = speed_abs_float((float)error_far);

  // 约束菜单速度参数，保证：直道速度 >= 弯道速度 >= 0。
  straight_speed = speed_straight_speed;
  corner_speed = speed_corner_speed;
  if (straight_speed < 0) straight_speed = 0;
  if (corner_speed > straight_speed) corner_speed = straight_speed;
  if (corner_speed < 0) corner_speed = 0;

  if (speed_state == SPEED_STATE_STRAIGHT)
  {
    // 直道状态：偏差连续达到阈值才切换到弯道，过滤出弯后的单帧抖动。
    if (line_error >= SPEED_ENTER_LINE_PX)
    {
      speed_corner_frame_count++;
      if (speed_corner_confirm_frames <= 0 ||
          speed_corner_frame_count >= speed_corner_confirm_frames)
      {
        speed_state = SPEED_STATE_CORNER;
        speed_corner_frame_count = 0;
        speed_straight_frame_count = 0;
      }
    }
    else
    {
      speed_corner_frame_count = 0;
    }
  }
  else
  {
    // 弯道状态：图像中线偏差足够小，才累计直道确认帧。
    speed_corner_frame_count = 0;
    if (line_error <= SPEED_EXIT_LINE_PX)
    {
      speed_straight_frame_count++;
      if (speed_straight_confirm_frames <= 0 ||
          speed_straight_frame_count >= speed_straight_confirm_frames)
      {
        speed_state = SPEED_STATE_STRAIGHT;
        speed_straight_frame_count = 0;
      }
    }
    else
    {
      speed_straight_frame_count = 0;
    }
  }

  // 使用完成滞回判断后的道路状态，切换方向陀螺仪抑制系数。
  Kgyro_steer = (speed_state == SPEED_STATE_STRAIGHT) ?
                speed_straight_kgyro_steer : SPEED_CORNER_KGYRO_STEER;

  // 状态只选择两档目标速度，不再计算curve_score或做速度插值。
  target_speed = (speed_state == SPEED_STATE_STRAIGHT) ?
                 (float)straight_speed : (float)corner_speed;
  accel_step = (speed_accel_step > 0.0f) ? speed_accel_step : 0.0f;
  decel_step = (speed_decel_step > 0.0f) ? speed_decel_step : 0.0f;

  // 快减慢加：入弯快速回到安全速度，确认出弯后再平缓提速。
  if (target_speed < speed_command)
  {
    speed_command -= decel_step;
    if (speed_command < target_speed) speed_command = target_speed;
  }
  else if (target_speed > speed_command)
  {
    speed_command += accel_step;
    if (speed_command > target_speed) speed_command = target_speed;
  }

  // 正速度加0.5后取整，得到交给现有电机控制代码的整数速度。
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
