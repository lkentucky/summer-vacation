#include "motor.h"

#define D 6.5   //轮子直径
#define PPR 1024 //编码器每转脉冲数
#define Kp 1.0f
#define Ki 0.1f
#define Kd 0.01f

int motor_speedl = 0;
int motor_speedr = 0;   
int encoder_diffl = 0;
int encoder_diffr = 0;
float real_speedl = 0.0f;
float real_speedr = 0.0f;

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


//pid闭环控制电机转速
void motor_pid_speedcontrol(float vl, float vr)
{
    static float errk_1 = 0, errk_2 = 0;       // e(k-1), e(k-2)
    static float control_effortl = 0, control_effortr = 0;                 // u(k-1)

    float errl = vl - (float)real_speedl;  // 计算左轮速度误差
    float errr = vr - (float)real_speedr;  // 计算右轮速度误差 

    float deltal = Kp * (errl - errk_1)
                + Ki * errl
                + Kd * (errl - 2*errk_1 + errk_2);

    float deltar = Kp * (errr - errk_1)
                + Ki * errr
                + Kd * (errr - 2*errk_1 + errk_2);  

    errk_2 = errk_1;
    errk_1 = errl;
    control_effortl = control_effortl + deltal;               // u(k) = u(k-1) + Δu

    errk_2 = errk_1;
    errk_1 = errr;
    control_effortr = control_effortr + deltar;               // u(k) = u(k-1) + Δu

    motorl_set_pwm(real_speedl+(int)control_effortl);  // 设置左轮PWM
    motorr_set_pwm(real_speedr+(int)control_effortr);  // 设置右轮PWM
}