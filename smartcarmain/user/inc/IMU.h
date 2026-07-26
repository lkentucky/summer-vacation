#ifndef _IMU_h
#define _IMU_h

#include "zf_common_headfile.h"
#include <stdbool.h>

#define IMU_LARGE_ROTATION_THRESHOLD  (80.0f)   // deg/s, Z轴角速率超过此值认为大幅转动
#define IMU_GYRO_BIAS_SAMPLES         (200)     // 5ms间隔采样，约1秒
#define IMU_GYRO_CAL_MAX_SPAN_DPS     (8.0f)    // 校准期间任一轴波动过大则认为车未静止

// Mahony融合参数：加速度约束roll/pitch，磁力计约束yaw。
#define IMU_MAHONY_KP_ACCEL           (1.50f)
#define IMU_MAHONY_KP_MAG             (0.50f)
#define IMU_MAHONY_KI                 (0.03f)

// 磁力计受电机干扰严重时设为0，算法会自动退化为六轴。
#define IMU_USE_MAGNETOMETER          (0)

// 磁力计标定：calibrated = (raw - offset) * scale。
// 默认值可以运行；要获得准确yaw，需要根据实车标定结果填写。
#define IMU_MAG_OFFSET_X              (0.0f)
#define IMU_MAG_OFFSET_Y              (0.0f)
#define IMU_MAG_OFFSET_Z              (0.0f)
#define IMU_MAG_SCALE_X               (1.0f)
#define IMU_MAG_SCALE_Y               (1.0f)
#define IMU_MAG_SCALE_Z               (1.0f)

extern bool  imu_ready;              // IMU初始化成功标志
extern bool  imu_mag_valid;          // 当前磁场数据通过幅值检查
extern float imu_gyro_z_dps;         // Z轴角速率滤波前数据 (deg/s)
extern float imu_gyro_z_dps_filter;  // Z轴角速率低通滤波后数据 (deg/s)，用于方向抑制
extern float imu_pitch;              // 俯仰角 (deg)
extern float imu_roll;               // 横滚角 (deg)
extern float imu_yaw;                // 相对启动方向的偏航角 (deg)，范围[-180, 180]
extern float Kgyro_steer;            // 方向陀螺抑制系数，0表示关闭，可正可负用于适配安装方向

uint8 imu_init(void);
void  imu_update(void);
float imu_get_steer_damping(void);
bool  imu_is_large_rotation(void);

#endif
