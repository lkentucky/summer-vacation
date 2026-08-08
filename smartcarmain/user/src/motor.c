#include "motor.h"
#include "IMU.h"
#include "isr.h"

#define D 6.5   //轮子直径
#define PPR 1024 //编码器每转脉冲数
#define STEER_MAX_RATIO       (0.85f)   // 当前基础速度允许的最大左右轮差速比例，单位：无量纲。
#define STEER_MAX_STEP        (15.0f)   // 每10ms转向环更新允许的最大差速变化，单位cm/s。
#define VISION_D_FILTER_HZ      (6.0f)   // 视觉误差微分低通截止频率，单位Hz。
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
int run_base_speed = 280; //250// 菜单可调的启动/巡线速度，K4 启动时赋给 base_speed，
volatile uint8 joystick_control_active = 0;
volatile int joystick_turn_percent = 0;
volatile int joystick_forward_percent = 0;
static volatile float joystick_target_speedl = 0.0f;
static volatile float joystick_target_speedr = 0.0f;
static volatile uint32 joystick_last_packet_tick = 0;
// 视觉外环P系数：每1像素横向偏差产生多少期望角速度，单位(deg/s)/pixel。
float vision_yaw_kp = 4.0f;
// 视觉外环D系数：误差变化速度转换为期望角速度的系数，单位deg/pixel。
float vision_yaw_kd = 0.03f;
// 远点相对加权偏差的预瞄前馈系数，单位(deg/s)/pixel。
float vision_yaw_kff = 0.40f;
// 当前道路状态实际使用的角速度内环P系数，由速度状态机自动切换。
float yaw_rate_kp = 1.53f;//1.53
// 视觉外环允许输出的最大期望角速度绝对值，单位deg/s。
int yaw_rate_limit_dps = 180;
// 当前道路状态实际使用的角速度反馈方向/比例，由速度状态机在直道与弯道值之间切换。
float yaw_rate_feedback_sign = -1.01f;
// 视觉外环输出的期望角速度，主循环写入、10ms方向内环读取，单位deg/s。
volatile float yaw_rate_ref_dps = 0.0f;
// 角速度内环当前误差，供菜单观察，单位deg/s。
volatile float yaw_rate_error_dps = 0.0f;

static volatile int16 steer_error_weighted = 0; // 加权中线偏差，不是单独近点，由主循环按帧更新。
static volatile int16 steer_error_far = 0;      // 35~40行远点中线平均偏差，由主循环按帧更新。
static int16 vision_last_weighted_error = 0;    // 上一图像帧的加权偏差，用于计算真实视觉微分。
static float vision_error_rate_filter = 0.0f;      // 低通后的视觉误差变化速度，单位pixel/s。
static float vision_preview_error_filter = 0.0f;   // 低通后的“远点-加权偏差”预瞄增量，单位pixel。
static bool vision_last_error_valid = false;       // false表示尚无上一帧，首帧不计算微分。
static volatile uint32 vision_last_image_tick = 0; // 最近一次视觉外环更新时刻，单位2ms系统tick。
static float steer_last_output = 0.0f;             // 上一次左右轮差速指令，用于限制每10ms变化量。

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
float SPEED_ENTER_LINE_PX= (10.0f);
// 弯道状态下，最大中线偏差必须低于该值才可能判定出弯，单位：像素。
float SPEED_EXIT_LINE_PX= (10.0f);
// 角速度换向必须集中在该图像帧窗口内，才认为是快速左右摇摆。
#define SPEED_OSCILLATION_WINDOW_FRAMES (10)
// 摆动状态保持该帧数后进入直道，期间使用弯道安全速度和直道方向参数。
#define SPEED_OSCILLATION_HOLD_FRAMES   (8)
// 摆动状态至少保持该帧数后，才允许判断是否仍在真实弯道。
#define SPEED_OSCILLATION_MIN_HOLD_FRAMES (5)
// 摆动状态中线偏差大且角速度同向持续该帧数，直接返回弯道状态。
#define SPEED_OSCILLATION_CORNER_CONFIRM_FRAMES (3)

// 直道状态的目标速度，单位：cm/s。
int speed_straight_speed = 290;
// 弯道状态的目标速度，单位：cm/s；应设置为实车已验证的安全速度。
int speed_corner_speed = 235;
// 直道使用原来的角速度反馈方向/比例。
float speed_straight_yaw_feedback_sign = -1.01f;  //-1.01
// 弯道降低角速度反馈比例，避免影响弯道响应。
float speed_corner_yaw_feedback_sign = -0.46f;//-0.40
// 直道/出弯稳定阶段的角速度内环P系数。
float speed_straight_yaw_rate_kp = 1.38f;
// 弯道的角速度内环P系数。
float speed_corner_yaw_rate_kp = 1.46f;
// 直道/出弯稳定阶段直接使用的视觉外环P系数。
float speed_straight_vision_kp = 4.0f;
// 弯道直接使用的视觉外环P系数。
float speed_corner_vision_kp = 6.8f;
// 只有角速度绝对值达到该值时，其正负变化才计入摆动检测，避免零点噪声误触发。
float speed_oscillation_gyro_threshold = 15.0f;
// 在检测窗口内达到该换向次数后进入摆动抑制状态。
int speed_oscillation_reversal_required = 20;
// 当前速度状态；0是直道，1是弯道，复位时默认按直道处理。
int speed_state = SPEED_STATE_STRAIGHT;
// 最终提供给base_speed的整数速度指令，初始值为起步速度，单位：cm/s。
int speed_decision_speed = 8;
// 每处理一个图像帧，速度最多增加多少，单位：cm/s/帧。
float speed_accel_step = 8.0f;
// 每处理一个图像帧，速度最多降低多少，单位：cm/s/帧。
float speed_decel_step = 13.0f;
// 在弯道状态下，连续满足多少帧出弯条件后才切换到直道。
int speed_straight_confirm_frames = 2;
// 在直道状态下，连续满足多少帧入弯条件后才切换到弯道。
int speed_corner_confirm_frames = 1;

// 弯道状态下已经连续满足出弯条件的帧数，仅在本文件内部使用。
static int speed_straight_frame_count = 0;
// 直道状态下已经连续满足入弯条件的帧数，仅在本文件内部使用。
static int speed_corner_frame_count = 0;
// 弯道中首次出现出弯条件后置true；此阶段速度不变，但提前使用直道方向参数。
static bool speed_exit_stabilizing = false;
// 出弯稳定阶段连续重新满足明显弯道条件的帧数，用于防止单帧摇摆撤销稳定阶段。
static int speed_exit_cancel_frame_count = 0;
// 最近一次超过检测阈值的角速度符号：1为正，-1为负，0为尚无有效样本。
static int speed_oscillation_last_sign = 0;
// 当前检测窗口内已经出现的有效角速度换向次数。
static int speed_oscillation_reversal_count = 0;
// 从本轮第一次换向开始经过的图像帧数。
static int speed_oscillation_window_age = 0;
// 摆动抑制状态已经保持的图像帧数。
static int speed_oscillation_hold_count = 0;
// 摆动状态中用于确认真实弯道的上一帧有效角速度符号。
static int speed_oscillation_corner_last_sign = 0;
// 摆动状态中“偏差大且角速度同向”的连续确认帧数。
static int speed_oscillation_corner_frame_count = 0;
// 带加减速斜率限制的浮点速度指令，保留小数以避免每帧取整误差，单位：cm/s。
static float speed_command = 8.0f;

// 返回浮点数绝对值，使左右弯使用同一组判断阈值。
static float speed_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

// 每个图像帧检查一次角速度符号；在限定窗口内多次正负换向时返回true。
static bool speed_oscillation_detect(void)
{
  float gyro_z;        // 当前滤波后的Z轴角速度，单位deg/s。
  float threshold;     // 检查为非负数后的有效角速度阈值，单位deg/s。
  int current_sign;    // 当前有效角速度符号：1、-1或低于阈值时的0。
  int required_count;  // 检查后的触发换向次数。

  gyro_z = imu_gyro_z_dps_filter;
  threshold = speed_abs_float(speed_oscillation_gyro_threshold);
  required_count = speed_oscillation_reversal_required;
  current_sign = 0;

  if (gyro_z >= threshold)
    current_sign = 1;
  else if (gyro_z <= -threshold)
    current_sign = -1;

  if (speed_oscillation_reversal_count > 0)
  {
    speed_oscillation_window_age++;
    if (speed_oscillation_window_age > SPEED_OSCILLATION_WINDOW_FRAMES)
    {
      speed_oscillation_reversal_count = 0;
      speed_oscillation_window_age = 0;
    }
  }

  if (current_sign != 0)
  {
    if (speed_oscillation_last_sign != 0 && current_sign != speed_oscillation_last_sign)
    {
      if (speed_oscillation_reversal_count == 0)
        speed_oscillation_window_age = 0;
      speed_oscillation_reversal_count++;
      speed_oscillation_last_sign = current_sign;

      if (required_count <= 0 || speed_oscillation_reversal_count >= required_count)
      {
        speed_oscillation_reversal_count = 0;
        speed_oscillation_window_age = 0;
        return true;
      }
    }
    else
    {
      speed_oscillation_last_sign = current_sign;
    }
  }

  return false;
}

// 停车或控制器复位时，清除状态计数并恢复到直道状态。
void speed_decision_reset(void)
{
  int straight_speed = speed_straight_speed; // 检查为非负数后的直道速度，单位：cm/s。
  float startup_speed = speed_accel_step;   // 起步速度使用一帧加速量，后续继续逐帧加速。

  if (straight_speed < 0) straight_speed = 0;
  if (startup_speed < 0.0f) startup_speed = -startup_speed;
  if (startup_speed < 1.0f && straight_speed > 0) startup_speed = 1.0f;
  if (startup_speed > (float)straight_speed)
  {
    startup_speed = (float)straight_speed;
  }

  speed_state = SPEED_STATE_STRAIGHT;      // 起步直接使用直道状态。
  yaw_rate_feedback_sign = speed_straight_yaw_feedback_sign; // 复位时同步使用直道反馈值。
  yaw_rate_kp = speed_straight_yaw_rate_kp; // 复位时同步使用直道角速度内环P系数。
  vision_yaw_kp = speed_straight_vision_kp; // 复位时同步使用直道视觉P系数。
  speed_straight_frame_count = 0;         // 清除之前累计的直道帧。
  speed_corner_frame_count = 0;           // 清除之前累计的弯道帧。
  speed_exit_stabilizing = false;         // 清除出弯稳定阶段标志。
  speed_exit_cancel_frame_count = 0;      // 清除撤销稳定阶段的确认帧数。
  speed_oscillation_last_sign = 0;        // 清除上一有效角速度符号。
  speed_oscillation_reversal_count = 0;   // 清除摆动换向次数。
  speed_oscillation_window_age = 0;       // 清除摆动检测窗口年龄。
  speed_oscillation_hold_count = 0;       // 清除摆动状态保持帧数。
  speed_oscillation_corner_last_sign = 0; // 清除摆动状态的真实弯道方向记录。
  speed_oscillation_corner_frame_count = 0; // 清除摆动状态的真实弯道确认帧数。
  speed_command = startup_speed;          // 浮点指令回到起步速度。
  speed_decision_speed = (int)(startup_speed + 0.5f); // 对外整数指令同步复位。
}

// 每个新图像帧调用一次：更新状态机，再用快减慢加生成最终速度指令。
void speed_decision_update(void)
{
  int16 error_weighted;   // 加权中线相对图像中心的有符号偏差，单位：像素。
  int16 error_far;        // 远处中线相对图像中心的有符号偏差，单位：像素。
  float line_error;       // 加权与远点偏差绝对值中的较大者，单位：像素。
  float target_speed;     // 当前状态对应的目标速度，单位：cm/s。
  float accel_step;       // 检查为非负数后的本帧加速步长，单位：cm/s。
  float decel_step;       // 检查为非负数后的本帧减速步长，单位：cm/s。
  int straight_speed;     // 检查为非负数后的直道速度，单位：cm/s。
  int corner_speed;       // 检查范围后的弯道速度，单位：cm/s。

  // 停车时不累计状态，下一次起步从直道状态和直道速度开始。
  if (base_speed <= 0)
  {
    speed_decision_reset();
    return;
  }

  // 读取本帧加权与远点图像偏差，使用绝对值较大的一项判断弯道。
  error_weighted = steer_error_weighted;
  error_far = steer_error_far;
  line_error = speed_abs_float((float)error_weighted);
  if (speed_abs_float((float)error_far) > line_error)
    line_error = speed_abs_float((float)error_far);

  // 约束菜单速度参数，保证：直道速度 >= 弯道速度 >= 0。
  straight_speed = speed_straight_speed;
  corner_speed = speed_corner_speed;
  if (straight_speed < 0) straight_speed = 0;
  if (corner_speed > straight_speed) corner_speed = straight_speed;
  if (corner_speed < 0) corner_speed = 0;

  // 普通直道或弯道中检测到快速多次换向，就进入独立的摆动抑制状态。
  if (speed_state != SPEED_STATE_OSCILLATION && speed_oscillation_detect())
  {
    speed_state = SPEED_STATE_OSCILLATION;
    speed_oscillation_hold_count = 0;
    speed_oscillation_corner_last_sign = 0;
    speed_oscillation_corner_frame_count = 0;
    speed_straight_frame_count = 0;
    speed_corner_frame_count = 0;
    speed_exit_stabilizing = false;
    speed_exit_cancel_frame_count = 0;
  }

  if (speed_state == SPEED_STATE_OSCILLATION)
  {
    float gyro_threshold; // 检查为非负数后的角速度有效阈值，单位deg/s。
    int gyro_turn_sign;   // 当前有效角速度方向：1为正，-1为负，0为低于阈值。

    // 前4帧只负责抑制摇摆，之后才允许识别持续单向转弯并直接返回弯道。
    speed_oscillation_hold_count++;
    gyro_threshold = speed_abs_float(speed_oscillation_gyro_threshold);
    gyro_turn_sign = 0;
    if (imu_gyro_z_dps_filter >= gyro_threshold)
      gyro_turn_sign = 1;
    else if (imu_gyro_z_dps_filter <= -gyro_threshold)
      gyro_turn_sign = -1;

    if (speed_oscillation_hold_count >= SPEED_OSCILLATION_MIN_HOLD_FRAMES &&
        line_error >= SPEED_ENTER_LINE_PX && gyro_turn_sign != 0)
    {
      if (gyro_turn_sign == speed_oscillation_corner_last_sign)
      {
        speed_oscillation_corner_frame_count++;
      }
      else
      {
        speed_oscillation_corner_last_sign = gyro_turn_sign;
        speed_oscillation_corner_frame_count = 1;
      }

      if (speed_oscillation_corner_frame_count >=
          SPEED_OSCILLATION_CORNER_CONFIRM_FRAMES)
      {
        // 偏差持续很大且车辆持续单向旋转，说明仍在真实弯道，直接恢复弯道状态。
        speed_state = SPEED_STATE_CORNER;
        speed_oscillation_hold_count = 0;
        speed_oscillation_corner_last_sign = 0;
        speed_oscillation_corner_frame_count = 0;
        speed_oscillation_last_sign = 0;
        speed_oscillation_reversal_count = 0;
        speed_oscillation_window_age = 0;
        speed_straight_frame_count = 0;
        speed_corner_frame_count = 0;
      }
    }
    else
    {
      speed_oscillation_corner_last_sign = 0;
      speed_oscillation_corner_frame_count = 0;
    }

    if (speed_state == SPEED_STATE_OSCILLATION &&
        speed_oscillation_hold_count >= SPEED_OSCILLATION_HOLD_FRAMES)
    {
      speed_state = SPEED_STATE_STRAIGHT;
      speed_oscillation_hold_count = 0;
      speed_oscillation_corner_last_sign = 0;
      speed_oscillation_corner_frame_count = 0;
      speed_oscillation_last_sign = 0;
      speed_oscillation_reversal_count = 0;
      speed_oscillation_window_age = 0;
    }
  }
  else if (speed_state == SPEED_STATE_STRAIGHT)
  {
    speed_exit_stabilizing = false;
    speed_exit_cancel_frame_count = 0;
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
      speed_exit_stabilizing = true;      // 首帧出弯迹象就提前切换直道视觉P系数并加强角速度反馈。
      speed_exit_cancel_frame_count = 0;  // 当前满足出弯条件，不累计撤销帧数。
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

      if (speed_exit_stabilizing)
      {
        // 10~15像素属于滞回区，继续稳定；连续明显入弯才撤销出弯稳定阶段。
        if (line_error >= SPEED_ENTER_LINE_PX)
        {
          speed_exit_cancel_frame_count++;
          if (speed_corner_confirm_frames <= 0 ||
              speed_exit_cancel_frame_count >= speed_corner_confirm_frames)
          {
            speed_exit_stabilizing = false;
            speed_exit_cancel_frame_count = 0;
          }
        }
        else
        {
          speed_exit_cancel_frame_count = 0;
        }
      }
    }
  }

  // 正式直道或出弯稳定阶段都提前使用直道参数，但速度仍只由正式道路状态决定。
  if (speed_state == SPEED_STATE_STRAIGHT ||
      speed_state == SPEED_STATE_OSCILLATION || speed_exit_stabilizing)
  {
    yaw_rate_feedback_sign = speed_straight_yaw_feedback_sign;
    yaw_rate_kp = speed_straight_yaw_rate_kp;
    vision_yaw_kp = speed_straight_vision_kp;
  }
  else
  {
    yaw_rate_feedback_sign = speed_corner_yaw_feedback_sign;
    yaw_rate_kp = speed_corner_yaw_rate_kp;
    vision_yaw_kp = speed_corner_vision_kp;
  }

  // 状态只选择两档目标速度，不再计算curve_score或做速度插值。
  target_speed = (speed_state == SPEED_STATE_STRAIGHT) ?
                 (float)straight_speed : (float)corner_speed; // 摆动状态仍使用弯道安全速度。
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

// 每个新图像帧调用一次：加权偏差用于主反馈，远点与加权偏差之差用于预瞄前馈。
// error_weighted是多行加权结果，不是严格几何近点，因此前馈量按预瞄增量而非远近两点斜率处理。
void steering_set_image_error(int16 error_weighted, int16 error_far, float image_dt_s)
{
  float dt;               // 检查范围后的实际图像周期，单位s。
  float raw_error_rate;   // 本帧横向偏差变化速度，单位pixel/s。
  float raw_preview_error;// 本帧远点相对加权偏差的预瞄增量，单位pixel。
  float filter_tau;       // 视觉微分和预瞄前馈共用的一阶低通时间常数，单位s。
  float filter_alpha;     // 一阶低通中上一结果所占比例。
  float yaw_limit;        // 检查为非负数后的期望角速度限幅，单位deg/s。
  float yaw_ref;          // 本帧视觉反馈与预瞄前馈合成的期望角速度，单位deg/s。
  float image_error;      // 本帧用于主反馈的加权中线偏差，单位pixel。

  steer_error_weighted = error_weighted;
  steer_error_far = error_far;

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
  yaw_ref = vision_yaw_kp * image_error +
            vision_yaw_kff * vision_preview_error_filter +
            vision_yaw_kd * vision_error_rate_filter;
  yaw_rate_ref_dps = motor_limit_float(yaw_ref, -yaw_limit, yaw_limit);
}

int16 steering_get_image_error(void)
{
  return steer_error_weighted;
}

void motor_joystick_stop(void)
{
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
// 由TIM6每10ms分频调用：角速度P内环跟踪视觉外环给出的期望角速度。
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
    steering = motor_limit_float(steering,
                                 steer_last_output - STEER_MAX_STEP,
                                 steer_last_output + STEER_MAX_STEP);

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
    steer_error_far = 0;
    vision_last_weighted_error = 0;
    vision_error_rate_filter = 0.0f;
    vision_preview_error_filter = 0.0f;
    vision_last_error_valid = false;
    vision_last_image_tick = g_sys_tick;
    yaw_rate_ref_dps = 0.0f;
    yaw_rate_error_dps = 0.0f;
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
