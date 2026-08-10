#include "motor.h"
#include "image.h"
#include "IMU.h"
#include "isr.h"

/* ------------------------------ 车辆与控制标定 ------------------------------ */
/* 驱动轮直径，单位cm；只用于把编码器计数换算成线速度。 */
#define WHEEL_DIAMETER_CM              (6.5f)
/* 车轮转一整圈对应的等效编码器计数；实测一圈后优先校准此值。 */
#define ENCODER_COUNTS_PER_WHEEL_REV   (1024.0f * 68.0f / 30.0f)
/* 单个电机允许输出的PWM绝对值上限；调大只会提高最大驱动力，不会提高控制精度。 */
#define MOTOR_PWM_LIMIT                (5000.0f)
/* 转向差速最大值占base_speed的比例，防止大转向量使单轮目标速度过度反向。 */
#define STEER_MAX_RATIO                (0.85f)
/* 超过该时间没有新视觉结果就把方向差速清零，单位ms。 */
#define VISION_STALE_TIMEOUT_MS        (120U)

/* ------------------------------ 左右轮速度PID ------------------------------ */
/* 速度环比例系数：增大可加快速度跟随，过大会引起轮速振荡或PWM跳变。 */
float Kp = 9.36f;
/* 速度环积分系数：用于消除稳态速度差，过大会积累过量并造成超调。 */
float Ki = 0.50f;
/* 速度环微分系数：抑制误差突变，过大会放大编码器噪声。 */
float Kd = 0.01f;

/* 左右轮换算后的整数速度，单位cm/s，主要供显示和调试。 */
int motor_speedl = 0;
int motor_speedr = 0;
/* 本次采样得到的左右编码器原始增量，单位count。 */
int encoder_diffl = 0;
int encoder_diffr = 0;
/* 固定PWM测试时累计的左右编码器计数，用于检查方向和硬件。 */
volatile int32 encoder_test_total_l = 0;
volatile int32 encoder_test_total_r = 0;
/* 左右轮滤波后的实际速度，单位cm/s。 */
float real_speedl = 0.0f;
float real_speedr = 0.0f;
/* 方向差速叠加后的左右轮目标速度，单位cm/s。 */
float target_speedl = 0.0f;
float target_speedr = 0.0f;
/* 当前实际下发的基础车速，单位cm/s；为0表示停车。 */
int base_speed = 0;
/* 关闭自动速度决策时使用的固定巡线速度，单位cm/s。 */
int run_base_speed = 280;

/* ------------------------------ 蓝牙摇杆状态 ------------------------------ */
/* 非0表示摇杆接管电机，接管期间绕过视觉巡线和自动速度决策。 */
volatile uint8 joystick_control_active = 0;
/* 摇杆转向/前进输入，范围-100~100。 */
volatile int joystick_turn_percent = 0;
volatile int joystick_forward_percent = 0;
/* 摇杆换算后的左右轮目标速度，单位cm/s。 */
static volatile float joystick_target_speedl = 0.0f;
static volatile float joystick_target_speedr = 0.0f;
/* 最近一次有效摇杆数据包的系统tick，用于通信超时停车。 */
static volatile uint32 joystick_last_packet_tick = 0;

/* ------------------------------ PPDD方向控制 ------------------------------ */
/* 中线加权偏差P系数，单位(cm/s)/pixel；入弯不够直接优先增大它。 */
float steer_ppdd_kp = 8.50f;
/* 相邻图像帧误差差的D系数；参考库默认为0，调大会抑制快速误差变化。 */
float steer_ppdd_kd = 0.00f;
/* Z轴角速度阻尼系数，单位(cm/s)/(deg/s)；车身旋转过快时压低差速。 */
float steer_ppdd_gyro_k = 0.50f;
/* 当前PPDD差速输出，单位cm/s；平方项KP2按用户要求不再计算。 */
volatile float steer_ppdd_output = 0.0f;

/* ------------------------------ 按加权偏差分档的四档速度 ------------------------------ */
/* speed_level=0：|steering_value|<1 pixel，单位cm/s。 */
int speed_straight_speed = 290;
/* speed_level=1：1~4 pixel。 */
int speed_mid_fast_speed = 270;
/* speed_level=2：4~8 pixel。 */
int speed_mid_slow_speed = 245;
/* speed_level=3：>=8 pixel。 */
int speed_corner_speed = 225;
/* 只是遥测标签：level 0/1显示STRAIGHT，level 2/3显示CORNER。 */
int speed_state = SPEED_STATE_STRAIGHT;
/* 当前速度档位0~3。 */
int speed_level = 0;
/* 分档后再叠加起步系数的基础速度，单位cm/s。 */
int speed_decision_speed = 5;
/* 起步线性斜坡总帧数；100fps时55帧约为550ms。 */
int speed_launch_frames = 55;
/* 当前起步斜坡帧计数，达到speed_launch_frames后进入正常分档速度。 */
int speed_launch_count = 0;

/* ------------------------------ 内部运行量（不需要菜单调节） ------------------------------ */
/* PPDD目标固定为0，这里保存当前误差(Target-Actual)和上帧误差。 */
static volatile float steer_error = 0.0f;
static float steer_last_error = 0.0f;
/* 最近一次视觉外环更新时间，用于视觉结果超时保护。 */
static volatile uint32 vision_last_image_tick = 0;

/* 左右轮增量式速度PID的前两次误差。 */
static float errl_k1 = 0.0f;
static float errl_k2 = 0.0f;
static float errr_k1 = 0.0f;
static float errr_k2 = 0.0f;
/* 左右轮速度PID累计控制量，最终限幅后转换为PWM。 */
static float control_effortl = 0.0f;
static float control_effortr = 0.0f;

static float motor_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float motor_limit_float(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static int motor_limit_int(int value, int lower, int upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

void speed_decision_reset(void)
{
    int straight = (speed_straight_speed > 0) ? speed_straight_speed : 0;
    int frames = (speed_launch_frames > 0) ? speed_launch_frames : 1;
    int startup = straight / frames;

    if (startup < 1 && straight > 0) startup = 1;
    speed_level = 0;
    speed_state = SPEED_STATE_STRAIGHT;
    speed_launch_count = 0;
    speed_decision_speed = startup;
}

void speed_decision_update(void)
{
    int candidate;
    int speeds[4];
    int target;
    float error_abs = motor_abs_float(track_steering_value);

    if (base_speed <= 0)
    {
        speed_decision_reset();
        return;
    }

    /* 与参考库一致：只看加权中线偏差的绝对值选速。 */
    if (error_abs < 1.0f) candidate = 0;
    else if (error_abs < 4.0f) candidate = 1;
    else if (error_abs < 8.0f) candidate = 2;
    else candidate = 3;
    speed_level = candidate;

    speeds[0] = (speed_straight_speed > 0) ? speed_straight_speed : 0;
    speeds[1] = motor_limit_int(speed_mid_fast_speed, 0, speeds[0]);
    speeds[2] = motor_limit_int(speed_mid_slow_speed, 0, speeds[1]);
    speeds[3] = motor_limit_int(speed_corner_speed, 0, speeds[2]);
    target = speeds[speed_level];

    speed_state = (speed_level <= 1) ? SPEED_STATE_STRAIGHT : SPEED_STATE_CORNER;

    /* 发车时把当前档位速度乘以线性因子；55帧@100fps约550ms。 */
    if (speed_launch_frames > 0 && speed_launch_count < speed_launch_frames)
    {
        speed_launch_count++;
        target = target * speed_launch_count / speed_launch_frames;
        if (target < 1 && speeds[speed_level] > 0) target = 1;
    }
    speed_decision_speed = target;
}

void steering_set_image_error(float steering_value)
{
    float error = -steering_value; /* Target(0) - Actual(加权中线偏差) */

    steer_error = error;
    steer_ppdd_output = error * steer_ppdd_kp +
                        (error - steer_last_error) * steer_ppdd_kd -
                        imu_gyro_z_dps_filter * steer_ppdd_gyro_k;
    steer_last_error = error;
    vision_last_image_tick = g_sys_tick;
}

int16 steering_get_image_error(void)
{
    return (int16)(-steer_error);
}

int16 steering_get_far_error(void)
{
    return 0;
}

int16 steering_get_speed_error(void)
{
    return (int16)(motor_abs_float(track_steering_value) * 10.0f + 0.5f);
}

void motor_joystick_stop(void)
{
    joystick_control_active = 0U;
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

    turn_percent = motor_limit_int(turn_percent, -100, 100);
    forward_percent = motor_limit_int(forward_percent, -100, 100);
    if (turn_percent >= -JOYSTICK_DEADZONE && turn_percent <= JOYSTICK_DEADZONE) turn_percent = 0;
    if (forward_percent >= -JOYSTICK_DEADZONE && forward_percent <= JOYSTICK_DEADZONE) forward_percent = 0;

    if (!joystick_control_active) motor_pid_reset();
    base_speed = 0;
    joystick_turn_percent = turn_percent;
    joystick_forward_percent = forward_percent;
    forward_speed = (float)forward_percent * JOYSTICK_MAX_SPEED_CM_S / 100.0f;
    turn_speed = (float)turn_percent * JOYSTICK_MAX_SPEED_CM_S * JOYSTICK_TURN_RATIO / 100.0f;
    joystick_target_speedl = motor_limit_float(forward_speed + turn_speed,
                                               -JOYSTICK_MAX_SPEED_CM_S,
                                                JOYSTICK_MAX_SPEED_CM_S);
    joystick_target_speedr = motor_limit_float(forward_speed - turn_speed,
                                               -JOYSTICK_MAX_SPEED_CM_S,
                                                JOYSTICK_MAX_SPEED_CM_S);
    joystick_last_packet_tick = g_sys_tick;
    joystick_control_active = 1U;
}

void steering_control_update(void)
{
    if (joystick_control_active)
    {
        uint32 timeout_ticks = (JOYSTICK_TIMEOUT_MS + SYS_TICK_MS - 1U) / SYS_TICK_MS;
        if (g_sys_tick - joystick_last_packet_tick >= timeout_ticks)
        {
            motor_joystick_stop();
            return;
        }
        steer_ppdd_output = 0.0f;
        target_speedl = joystick_target_speedl;
        target_speedr = joystick_target_speedr;
        return;
    }

    if (base_speed > 0)
    {
        float steering = steer_ppdd_output;
        float max_steering;

        if ((g_sys_tick - vision_last_image_tick) * SYS_TICK_MS > VISION_STALE_TIMEOUT_MS)
        {
            steering = 0.0f;
            steer_ppdd_output = 0.0f;
        }
        max_steering = (float)base_speed * STEER_MAX_RATIO;
        steering = motor_limit_float(steering, -max_steering, max_steering);
        target_speedl = (float)base_speed - steering;
        target_speedr = (float)base_speed + steering;
    }
    else
    {
        steer_ppdd_output = 0.0f;
        target_speedl = 0.0f;
        target_speedr = 0.0f;
    }
}

void motor_init(void)
{
    gpio_init(MOTORL_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTORR_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTORL_PWM, 10000, 0);
    pwm_init(MOTORR_PWM, 10000, 0);
}

void motorl_set_pwm(int lpwm)
{
    lpwm = motor_limit_int(lpwm, -10000, 10000);
    if (lpwm >= 0)
    {
        gpio_high(MOTORL_DIR);
        pwm_set_duty(MOTORL_PWM, lpwm);
    }
    else
    {
        gpio_low(MOTORL_DIR);
        pwm_set_duty(MOTORL_PWM, -lpwm);
    }
}

void motorr_set_pwm(int rpwm)
{
    rpwm = motor_limit_int(rpwm, -10000, 10000);
    if (rpwm >= 0)
    {
        gpio_low(MOTORR_DIR);
        pwm_set_duty(MOTORR_PWM, rpwm);
    }
    else
    {
        gpio_high(MOTORR_DIR);
        pwm_set_duty(MOTORR_PWM, -rpwm);
    }
}

void init_encoder(void)
{
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7);
}

void get_motor_speed(void)
{
    float circumference = WHEEL_DIAMETER_CM * PI;
    real_speedl = -(float)motor_speedl * circumference / ENCODER_COUNTS_PER_WHEEL_REV;
    real_speedr =  (float)motor_speedr * circumference / ENCODER_COUNTS_PER_WHEEL_REV;
}

void motor_pid_reset(void)
{
    errl_k1 = errl_k2 = 0.0f;
    errr_k1 = errr_k2 = 0.0f;
    control_effortl = control_effortr = 0.0f;
    steer_error = 0.0f;
    steer_last_error = 0.0f;
    vision_last_image_tick = g_sys_tick;
    steer_ppdd_output = 0.0f;
    speed_decision_reset();
}

void motor_pid_speedcontrol(void)
{
    float errl = target_speedl - real_speedl;
    float errr = target_speedr - real_speedr;
    float deltal = Kp * (errl - errl_k1) + Ki * errl +
                   Kd * (errl - 2.0f * errl_k1 + errl_k2);
    float deltar = Kp * (errr - errr_k1) + Ki * errr +
                   Kd * (errr - 2.0f * errr_k1 + errr_k2);

    errl_k2 = errl_k1;
    errl_k1 = errl;
    errr_k2 = errr_k1;
    errr_k1 = errr;
    control_effortl = motor_limit_float(control_effortl + deltal,
                                        -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    control_effortr = motor_limit_float(control_effortr + deltar,
                                        -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    motorl_set_pwm((int)control_effortl);
    motorr_set_pwm((int)control_effortr);
}
