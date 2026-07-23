#ifndef _IMU_h
#define _IMU_h

#include "zf_common_headfile.h"
#include <stdbool.h>

#define IMU_LARGE_ROTATION_THRESHOLD  (80.0f)   // deg/s, Z轴角速率超过此值认为大幅转动
#define IMU_GYRO_BIAS_SAMPLES         (200)     // 零偏校准采样数
#define IMU_COMPLEMENTARY_ALPHA       (0.98f)   // 互补滤波系数

extern bool  imu_ready;              // IMU 初始化成功标志
extern float imu_gyro_z_dps;         // Z轴角速率原始滤波前数据 (deg/s)
extern float imu_gyro_z_dps_filter;  // Z轴角速率低通滤波后数据 (deg/s)，用于方向抑制
extern float imu_pitch;              // 俯仰角 (deg)
extern float imu_roll;               // 横滚角 (deg)
extern float imu_yaw;                // 偏航角 (deg)，仅积分
extern float Kgyro_steer;            // 方向陀螺抑制系数，0表示关闭，可正可负用于适配安装方向

uint8 imu_init(void);
void  imu_update(void);
float imu_get_steer_damping(void);
bool  imu_is_large_rotation(void);

#endif
