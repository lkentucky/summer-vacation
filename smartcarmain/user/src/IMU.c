#include "IMU.h"
#include "isr.h"
#include <math.h>

#define DEG_TO_RAD                    (0.01745329252f)
#define RAD_TO_DEG                    (57.29577951f)
#define IMU_NOMINAL_DT                (1.0f / 208.0f)
#define IMU_MAX_DT                    (0.030f)
#define IMU_GYRO_Z_LPF_HZ             (15.0f)
#define IMU_ACCEL_NORM_MIN            (0.85f)
#define IMU_ACCEL_NORM_MAX            (1.15f)
#define IMU_MAG_NORM_MIN_RATIO        (0.65f)
#define IMU_MAG_NORM_MAX_RATIO        (1.35f)
#define IMU_MAG_UPDATE_TICKS          (5U)       // 5 * 2ms = 10ms，匹配100Hz磁力计
#define IMU_INTEGRAL_LIMIT            (0.0873f)  // 最大约5deg/s的零偏修正
#define IMU_EPSILON                   (1.0e-9f)

bool  imu_ready             = false;
bool  imu_mag_valid         = false;
float imu_gyro_z_dps        = 0.0f;
float imu_gyro_z_dps_filter = 0.0f;
float imu_pitch             = 0.0f;
float imu_roll              = 0.0f;
float imu_yaw               = 0.0f;
float Kgyro_steer           = 1.53f; // 默认按弯道系数，运行时随直道/弯道状态切换

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integral_x = 0.0f;
static float integral_y = 0.0f;
static float integral_z = 0.0f;

static float mag_x = 0.0f;
static float mag_y = 0.0f;
static float mag_z = 0.0f;
#if IMU_USE_MAGNETOMETER
static float mag_reference_norm = 0.0f;
#endif
static float yaw_zero_deg = 0.0f;

static int16 last_raw_gyro_x = 0;
static int16 last_raw_gyro_y = 0;
static int16 last_raw_gyro_z = 0;
static bool last_raw_gyro_valid = false;
static uint32 last_update_tick = 0;
static uint32 last_mag_tick = 0;

static float imu_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float imu_clamp_float(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static float imu_wrap_180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void imu_normalize_quaternion(void)
{
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    if (norm <= IMU_EPSILON)
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        return;
    }

    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

static void imu_quaternion_from_euler(float roll, float pitch, float yaw)
{
    float half_roll = 0.5f * roll;
    float half_pitch = 0.5f * pitch;
    float half_yaw = 0.5f * yaw;
    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
    imu_normalize_quaternion();
}

static float imu_quaternion_yaw_deg(void)
{
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

static void imu_update_euler(void)
{
    float pitch_sin = 2.0f * (q0 * q2 - q3 * q1);

    pitch_sin = imu_clamp_float(pitch_sin, -1.0f, 1.0f);
    imu_roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    imu_pitch = asinf(pitch_sin) * RAD_TO_DEG;
    imu_yaw = imu_wrap_180(imu_quaternion_yaw_deg() - yaw_zero_deg);
}

static float imu_limit_integral(float value)
{
    return imu_clamp_float(value, -IMU_INTEGRAL_LIMIT, IMU_INTEGRAL_LIMIT);
}

#if IMU_USE_MAGNETOMETER
static void imu_get_calibrated_mag(float *mx, float *my, float *mz)
{
    imu963ra_get_mag();
    *mx = (imu963ra_mag_transition(imu963ra_mag_x) - IMU_MAG_OFFSET_X) * IMU_MAG_SCALE_X;
    *my = (imu963ra_mag_transition(imu963ra_mag_y) - IMU_MAG_OFFSET_Y) * IMU_MAG_SCALE_Y;
    *mz = (imu963ra_mag_transition(imu963ra_mag_z) - IMU_MAG_OFFSET_Z) * IMU_MAG_SCALE_Z;
}
#endif

static bool imu_refresh_mag(void)
{
#if IMU_USE_MAGNETOMETER
    float norm;

    imu_get_calibrated_mag(&mag_x, &mag_y, &mag_z);
    norm = sqrtf(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);

    if (norm <= IMU_EPSILON)
    {
        return false;
    }

    if (mag_reference_norm <= IMU_EPSILON)
    {
        mag_reference_norm = norm;
    }

    if (norm < mag_reference_norm * IMU_MAG_NORM_MIN_RATIO ||
        norm > mag_reference_norm * IMU_MAG_NORM_MAX_RATIO)
    {
        return false;
    }

    mag_x /= norm;
    mag_y /= norm;
    mag_z /= norm;
    return true;
#else
    return false;
#endif
}

static bool imu_calibrate_gyro(void)
{
    int32 sum_x = 0;
    int32 sum_y = 0;
    int32 sum_z = 0;
    int16 min_x = 0;
    int16 min_y = 0;
    int16 min_z = 0;
    int16 max_x = 0;
    int16 max_y = 0;
    int16 max_z = 0;
    uint16 i;

    for (i = 0; i < IMU_GYRO_BIAS_SAMPLES; i++)
    {
        imu963ra_get_gyro();

        if (i == 0)
        {
            min_x = max_x = imu963ra_gyro_x;
            min_y = max_y = imu963ra_gyro_y;
            min_z = max_z = imu963ra_gyro_z;
        }
        else
        {
            if (imu963ra_gyro_x < min_x) min_x = imu963ra_gyro_x;
            if (imu963ra_gyro_x > max_x) max_x = imu963ra_gyro_x;
            if (imu963ra_gyro_y < min_y) min_y = imu963ra_gyro_y;
            if (imu963ra_gyro_y > max_y) max_y = imu963ra_gyro_y;
            if (imu963ra_gyro_z < min_z) min_z = imu963ra_gyro_z;
            if (imu963ra_gyro_z > max_z) max_z = imu963ra_gyro_z;
        }

        sum_x += imu963ra_gyro_x;
        sum_y += imu963ra_gyro_y;
        sum_z += imu963ra_gyro_z;
        system_delay_ms(5);
    }

    if (imu_abs_float(imu963ra_gyro_transition(max_x) - imu963ra_gyro_transition(min_x)) > IMU_GYRO_CAL_MAX_SPAN_DPS ||
        imu_abs_float(imu963ra_gyro_transition(max_y) - imu963ra_gyro_transition(min_y)) > IMU_GYRO_CAL_MAX_SPAN_DPS ||
        imu_abs_float(imu963ra_gyro_transition(max_z) - imu963ra_gyro_transition(min_z)) > IMU_GYRO_CAL_MAX_SPAN_DPS)
    {
        return false;
    }

    gyro_bias_x = imu963ra_gyro_transition((int16)(sum_x / IMU_GYRO_BIAS_SAMPLES));
    gyro_bias_y = imu963ra_gyro_transition((int16)(sum_y / IMU_GYRO_BIAS_SAMPLES));
    gyro_bias_z = imu963ra_gyro_transition((int16)(sum_z / IMU_GYRO_BIAS_SAMPLES));

    last_raw_gyro_x = imu963ra_gyro_x;
    last_raw_gyro_y = imu963ra_gyro_y;
    last_raw_gyro_z = imu963ra_gyro_z;
    last_raw_gyro_valid = true;
    return true;
}

static void imu_initialize_attitude(void)
{
    float ax;
    float ay;
    float az;
    float acc_norm;
    float roll;
    float pitch;
    float yaw = 0.0f;

    imu963ra_get_acc();
    ax = imu963ra_acc_transition(imu963ra_acc_x);
    ay = imu963ra_acc_transition(imu963ra_acc_y);
    az = imu963ra_acc_transition(imu963ra_acc_z);
    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    if (acc_norm <= IMU_EPSILON)
    {
        imu_quaternion_from_euler(0.0f, 0.0f, 0.0f);
        return;
    }

    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    imu_mag_valid = imu_refresh_mag();
    if (imu_mag_valid)
    {
        float sin_roll = sinf(roll);
        float cos_roll = cosf(roll);
        float sin_pitch = sinf(pitch);
        float cos_pitch = cosf(pitch);
        float horizontal_x = mag_x * cos_pitch + mag_z * sin_pitch;
        float horizontal_y = mag_x * sin_roll * sin_pitch +
                             mag_y * cos_roll -
                             mag_z * sin_roll * cos_pitch;

        yaw = atan2f(-horizontal_y, horizontal_x);
    }

    imu_quaternion_from_euler(roll, pitch, yaw);
}

uint8 imu_init(void)
{
    imu_ready = false;
    imu_mag_valid = false;
    last_raw_gyro_valid = false;

    if (imu963ra_init())
    {
        return 1;
    }

    // 校准期间车辆必须静止；5ms间隔保证样本与208Hz陀螺输出基本同步。
    if (!imu_calibrate_gyro())
    {
        return 1;
    }

    integral_x = 0.0f;
    integral_y = 0.0f;
    integral_z = 0.0f;
#if IMU_USE_MAGNETOMETER
    mag_reference_norm = 0.0f;
#endif
    imu_initialize_attitude();

    yaw_zero_deg = imu_quaternion_yaw_deg();
    imu_gyro_z_dps = 0.0f;
    imu_gyro_z_dps_filter = 0.0f;
    last_update_tick = g_sys_tick;
    last_mag_tick = g_sys_tick;
    imu_update_euler();

    imu_ready = true;
    return 0;
}

static void imu_mahony_update(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt)
{
    float acc_error_x = 0.0f;
    float acc_error_y = 0.0f;
    float acc_error_z = 0.0f;
    float mag_error_x = 0.0f;
    float mag_error_y = 0.0f;
    float mag_error_z = 0.0f;
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    bool accel_valid = (acc_norm >= IMU_ACCEL_NORM_MIN && acc_norm <= IMU_ACCEL_NORM_MAX);
    bool correction_valid = false;

    if (accel_valid)
    {
        float half_vx;
        float half_vy;
        float half_vz;

        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        half_vx = q1 * q3 - q0 * q2;
        half_vy = q0 * q1 + q2 * q3;
        half_vz = q0 * q0 - 0.5f + q3 * q3;

        acc_error_x = ay * half_vz - az * half_vy;
        acc_error_y = az * half_vx - ax * half_vz;
        acc_error_z = ax * half_vy - ay * half_vx;
        correction_valid = true;
    }

    if (imu_mag_valid)
    {
        float hx;
        float hy;
        float bx;
        float bz;
        float half_wx;
        float half_wy;
        float half_wz;

        hx = 2.0f * (mag_x * (0.5f - q2 * q2 - q3 * q3) +
                     mag_y * (q1 * q2 - q0 * q3) +
                     mag_z * (q1 * q3 + q0 * q2));
        hy = 2.0f * (mag_x * (q1 * q2 + q0 * q3) +
                     mag_y * (0.5f - q1 * q1 - q3 * q3) +
                     mag_z * (q2 * q3 - q0 * q1));
        bx = 0.5f * sqrtf(hx * hx + hy * hy);
        bz = mag_x * (q1 * q3 - q0 * q2) +
             mag_y * (q2 * q3 + q0 * q1) +
             mag_z * (0.5f - q1 * q1 - q2 * q2);

        half_wx = bx * (0.5f - q2 * q2 - q3 * q3) +
                  bz * (q1 * q3 - q0 * q2);
        half_wy = bx * (q1 * q2 - q0 * q3) +
                  bz * (q0 * q1 + q2 * q3);
        half_wz = bx * (q0 * q2 + q1 * q3) +
                  bz * (0.5f - q1 * q1 - q2 * q2);

        mag_error_x = mag_y * half_wz - mag_z * half_wy;
        mag_error_y = mag_z * half_wx - mag_x * half_wz;
        mag_error_z = mag_x * half_wy - mag_y * half_wx;
        correction_valid = true;
    }

    if (correction_valid)
    {
        float total_error_x = acc_error_x + mag_error_x;
        float total_error_y = acc_error_y + mag_error_y;
        float total_error_z = acc_error_z + mag_error_z;

        integral_x = imu_limit_integral(integral_x + IMU_MAHONY_KI * total_error_x * dt);
        integral_y = imu_limit_integral(integral_y + IMU_MAHONY_KI * total_error_y * dt);
        integral_z = imu_limit_integral(integral_z + IMU_MAHONY_KI * total_error_z * dt);

        gx += IMU_MAHONY_KP_ACCEL * acc_error_x +
              IMU_MAHONY_KP_MAG * mag_error_x + integral_x;
        gy += IMU_MAHONY_KP_ACCEL * acc_error_y +
              IMU_MAHONY_KP_MAG * mag_error_y + integral_y;
        gz += IMU_MAHONY_KP_ACCEL * acc_error_z +
              IMU_MAHONY_KP_MAG * mag_error_z + integral_z;
    }

    {
        float old_q0 = q0;
        float old_q1 = q1;
        float old_q2 = q2;
        float old_q3 = q3;
        float half_dt = 0.5f * dt;

        q0 = old_q0 + (-old_q1 * gx - old_q2 * gy - old_q3 * gz) * half_dt;
        q1 = old_q1 + ( old_q0 * gx + old_q2 * gz - old_q3 * gy) * half_dt;
        q2 = old_q2 + ( old_q0 * gy - old_q1 * gz + old_q3 * gx) * half_dt;
        q3 = old_q3 + ( old_q0 * gz + old_q1 * gy - old_q2 * gx) * half_dt;
    }

    imu_normalize_quaternion();
}

void imu_update(void)
{
    uint32 now;
    float dt;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float filter_tau;
    float filter_alpha;

    if (!imu_ready)
    {
        return;
    }

    imu963ra_get_gyro();

    // 主循环约250Hz轮询，传感器为208Hz；完全相同的三轴原始值视为旧样本，不重复积分。
    if (last_raw_gyro_valid &&
        imu963ra_gyro_x == last_raw_gyro_x &&
        imu963ra_gyro_y == last_raw_gyro_y &&
        imu963ra_gyro_z == last_raw_gyro_z)
    {
        return;
    }

    last_raw_gyro_x = imu963ra_gyro_x;
    last_raw_gyro_y = imu963ra_gyro_y;
    last_raw_gyro_z = imu963ra_gyro_z;
    last_raw_gyro_valid = true;

    now = g_sys_tick;
    dt = (float)(now - last_update_tick) * SYS_TICK_SEC;
    last_update_tick = now;

    // 主循环若长时间阻塞，当前样本不能代表整段空窗，使用标称周期避免姿态突跳。
    if (dt <= 0.0f || dt > IMU_MAX_DT)
    {
        dt = IMU_NOMINAL_DT;
    }

    gx = imu963ra_gyro_transition(imu963ra_gyro_x) - gyro_bias_x;
    gy = imu963ra_gyro_transition(imu963ra_gyro_y) - gyro_bias_y;
    gz = imu963ra_gyro_transition(imu963ra_gyro_z) - gyro_bias_z;

    imu_gyro_z_dps = gz;
    filter_tau = 1.0f / (2.0f * 3.14159265359f * IMU_GYRO_Z_LPF_HZ);
    filter_alpha = filter_tau / (filter_tau + dt);
    imu_gyro_z_dps_filter = filter_alpha * imu_gyro_z_dps_filter +
                            (1.0f - filter_alpha) * gz;

    imu963ra_get_acc();
    ax = imu963ra_acc_transition(imu963ra_acc_x);
    ay = imu963ra_acc_transition(imu963ra_acc_y);
    az = imu963ra_acc_transition(imu963ra_acc_z);

    if ((uint32)(now - last_mag_tick) >= IMU_MAG_UPDATE_TICKS)
    {
        last_mag_tick = now;
        imu_mag_valid = imu_refresh_mag();
    }

    imu_mahony_update(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                      ax, ay, az, dt);
    imu_update_euler();
}

float imu_get_steer_damping(void)
{
    if (!imu_ready)
    {
        return 0.0f;
    }

    return Kgyro_steer * imu_gyro_z_dps_filter;
}

bool imu_is_large_rotation(void)
{
    return (imu_abs_float(imu_gyro_z_dps_filter) > IMU_LARGE_ROTATION_THRESHOLD);
}
