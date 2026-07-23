#include "IMU.h"
#include "isr.h"
#include <math.h>

#define RAD_TO_DEG                 (57.29578f)
#define IMU_GYRO_Z_FILTER_ALPHA    (0.70f)   // 越大越平滑，方向抑制响应越慢

bool  imu_ready             = false;
float imu_gyro_z_dps        = 0.0f;
float imu_gyro_z_dps_filter = 0.0f;
float imu_pitch             = 0.0f;
float imu_roll              = 0.0f;
float imu_yaw               = 0.0f;
float Kgyro_steer           = 0.15f;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static uint32 last_update_tick = 0;

static float imu_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

uint8 imu_init(void)
{
    imu_ready = false;

    if (imu963ra_init())
    {
        return 1;
    }

    // 陀螺仪零偏校准：开机后车必须保持静止，采样取均值。
    int32 sum_x = 0;
    int32 sum_y = 0;
    int32 sum_z = 0;

    for (uint16 i = 0; i < IMU_GYRO_BIAS_SAMPLES; i++)
    {
        imu963ra_get_gyro();
        sum_x += imu963ra_gyro_x;
        sum_y += imu963ra_gyro_y;
        sum_z += imu963ra_gyro_z;
        system_delay_ms(2);
    }

    gyro_bias_x = imu963ra_gyro_transition((int16)(sum_x / IMU_GYRO_BIAS_SAMPLES));
    gyro_bias_y = imu963ra_gyro_transition((int16)(sum_y / IMU_GYRO_BIAS_SAMPLES));
    gyro_bias_z = imu963ra_gyro_transition((int16)(sum_z / IMU_GYRO_BIAS_SAMPLES));

    // 用加速度计初始化 pitch/roll。yaw 没有地磁修正时只做短时间积分，不用于巡线抑制。
    imu963ra_get_acc();
    float ax = imu963ra_acc_transition(imu963ra_acc_x);
    float ay = imu963ra_acc_transition(imu963ra_acc_y);
    float az = imu963ra_acc_transition(imu963ra_acc_z);

    imu_pitch = atan2f(ax, az) * RAD_TO_DEG;
    imu_roll  = atan2f(ay, az) * RAD_TO_DEG;
    imu_yaw   = 0.0f;
    imu_gyro_z_dps = 0.0f;
    imu_gyro_z_dps_filter = 0.0f;

    last_update_tick = g_sys_tick;
    imu_ready = true;
    return 0;
}

void imu_update(void)
{
    if (!imu_ready)
    {
        return;
    }

    uint32 now = g_sys_tick;
    float dt = (float)(now - last_update_tick) * SYS_TICK_SEC;
    last_update_tick = now;

    if (dt <= 0.0f || dt > 0.2f)
    {
        dt = SYS_TICK_SEC;
    }

    imu963ra_get_gyro();
    float gx = imu963ra_gyro_transition(imu963ra_gyro_x) - gyro_bias_x;
    float gy = imu963ra_gyro_transition(imu963ra_gyro_y) - gyro_bias_y;
    float gz = imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_bias_z;

    imu_gyro_z_dps = gz;
    imu_gyro_z_dps_filter = IMU_GYRO_Z_FILTER_ALPHA * imu_gyro_z_dps_filter +
                            (1.0f - IMU_GYRO_Z_FILTER_ALPHA) * gz;

    imu963ra_get_acc();
    float ax = imu963ra_acc_transition(imu963ra_acc_x);
    float ay = imu963ra_acc_transition(imu963ra_acc_y);
    float az = imu963ra_acc_transition(imu963ra_acc_z);

    // 姿态角解算：pitch/roll 用互补滤波，yaw 只由 Z 轴陀螺积分，长时间会漂移。
    imu_pitch = IMU_COMPLEMENTARY_ALPHA * (imu_pitch + gy * dt) +
                (1.0f - IMU_COMPLEMENTARY_ALPHA) * atan2f(ax, az) * RAD_TO_DEG;
    imu_roll  = IMU_COMPLEMENTARY_ALPHA * (imu_roll + gx * dt) +
                (1.0f - IMU_COMPLEMENTARY_ALPHA) * atan2f(ay, az) * RAD_TO_DEG;
    imu_yaw   = imu_yaw + gz * dt;
}

float imu_get_steer_damping(void)
{
    if (!imu_ready)
    {
        return 0.0f;
    }

    // 返回需要从视觉 steering 中减掉的抑制量。
    // 若实车发现摆动更大，把菜单里的 Kgyro_steer 调成负数即可反向。
    return Kgyro_steer * imu_gyro_z_dps_filter;
}

bool imu_is_large_rotation(void)
{
    return (imu_abs_float(imu_gyro_z_dps_filter) > IMU_LARGE_ROTATION_THRESHOLD);
}
