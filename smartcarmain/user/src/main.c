/*********************************************************************************************************************
 * MM32F327X-G8P F车走马观碑程序
 * 硬件范围：MT9V03X、双电机与双编码器、IMU963RA、IPS200、按键、HC-04蓝牙。
 * 巡线主链路按ASC-Summer26参考工程适配：逐行扫线、单边路宽恢复、
 * 近处90行加权偏差、PPDD方向差速、四档速度与起步斜坡；保留本车硬件和十字/斑马线接口。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "image.h"
#include "cross.h"
#include "IMU.h"
#include "motor.h"
#include "mymenu.h"
#include "key.h"
#include "bluetooth_app.h"
#include <string.h>
#include <stdbool.h>

#define IPS200_TYPE                     (IPS200_TYPE_SPI)
#define CAMERA_FPS_TARGET               100
#define CAMERA_EXPOSURE_TIME            200
#define CAMERA_GAIN_VALUE               45
#define IMAGE_PERIOD_MS_DEFAULT          2
#define IMAGE_PERIOD_MS_MIN              SYS_TICK_MS
#define IMAGE_PERIOD_MS_MAX             50
#define IMAGE_DISPLAY_SKIP_FRAMES        5
#define TRACK_LOST_CONFIRM_FRAMES        4
#define TRACK_START_GRACE_FRAMES        12
#define ZEBRA_CHECK_ROWS                 3
#define ZEBRA_PATTERN_REQUIRED           4
#define ZEBRA_CONFIRM_FRAMES             2
#define ZEBRA_RELEASE_FRAMES             5
#define ZEBRA_STOP_COUNT                 2
#define ZEBRA_STATE_SEARCH               0
#define ZEBRA_STATE_CONFIRM              1
#define ZEBRA_STATE_PASSING              2

int image_period_ms = IMAGE_PERIOD_MS_DEFAULT;
int image_frame_ms = 0;
int image_proc_ms = 0;
int image_fps = 0;
int image_wait_count = 0;
int zebra_cross_count = 0;
int zebra_transition_count = 0;
int zebra_match_row_count = 0;
int zebra_state = ZEBRA_STATE_SEARCH;

static bool zebra_latched = false;
static uint8 zebra_confirm_count = 0U;
static uint8 zebra_release_count = 0U;

static uint32 image_ms_to_ticks(int milliseconds)
{
    if (milliseconds < IMAGE_PERIOD_MS_MIN) milliseconds = IMAGE_PERIOD_MS_MIN;
    if (milliseconds > IMAGE_PERIOD_MS_MAX) milliseconds = IMAGE_PERIOD_MS_MAX;
    image_period_ms = milliseconds;
    return (uint32)((milliseconds + SYS_TICK_MS - 1) / SYS_TICK_MS);
}

static int zebra_count_row_patterns(uint8 row)
{
    int count = 0;
    uint16 col;

    for (col = 1; col < MT9V03X_W - 2; col++)
    {
        if (twovalues_image[row][col - 1] == 255U &&
            twovalues_image[row][col] == 0U &&
            twovalues_image[row][col + 1] == 0U)
            count++;
    }
    return count;
}

static bool zebra_guard_valid(void)
{
    int valid = 0;
    int row;

    for (row = 82; row <= 92; row++)
        if (left_line_valid[row] && right_line_valid[row] &&
            left_line[row] < right_line[row]) valid++;
    return valid >= 5;
}

static bool zebra_process(void)
{
    bool candidate;
    int max_patterns = 0;
    int row_index;

    zebra_match_row_count = 0;
    for (row_index = 0; row_index < ZEBRA_CHECK_ROWS; row_index++)
    {
        uint8 row = (uint8)(MT9V03X_H - 1 - row_index);
        int patterns = zebra_count_row_patterns(row);
        if (patterns > max_patterns) max_patterns = patterns;
        if (patterns >= ZEBRA_PATTERN_REQUIRED) zebra_match_row_count++;
    }
    zebra_transition_count = max_patterns;
    candidate = (zebra_match_row_count >= 2 && zebra_guard_valid());

    if (base_speed <= 0)
    {
        zebra_state = ZEBRA_STATE_SEARCH;
        zebra_latched = false;
        zebra_confirm_count = 0U;
        zebra_release_count = 0U;
        return false;
    }

    if (zebra_latched)
    {
        zebra_state = ZEBRA_STATE_PASSING;
        if (candidate) zebra_release_count = 0U;
        else if (++zebra_release_count >= ZEBRA_RELEASE_FRAMES)
        {
            zebra_latched = false;
            zebra_state = ZEBRA_STATE_SEARCH;
            zebra_release_count = 0U;
        }
        return false;
    }

    if (!candidate)
    {
        zebra_state = ZEBRA_STATE_SEARCH;
        zebra_confirm_count = 0U;
        return false;
    }

    zebra_state = ZEBRA_STATE_CONFIRM;
    if (++zebra_confirm_count < ZEBRA_CONFIRM_FRAMES) return false;
    zebra_confirm_count = 0U;
    zebra_latched = true;
    zebra_state = ZEBRA_STATE_PASSING;
    if (zebra_cross_count < ZEBRA_STOP_COUNT) zebra_cross_count++;
    return zebra_cross_count >= ZEBRA_STOP_COUNT;
}

static void car_stop(const char *reason)
{
    motor_joystick_stop();
    ips200_show_string(0, 288, reason);
}

static void reset_run_state(void)
{
    zebra_cross_count = 0;
    zebra_transition_count = 0;
    zebra_match_row_count = 0;
    zebra_state = ZEBRA_STATE_SEARCH;
    zebra_latched = false;
    zebra_confirm_count = 0U;
    zebra_release_count = 0U;
    cross_state_reset();
}

int main(void)
{
    uint32 last_key_tick = 0U;
    uint32 last_imu_tick = 0U;
    uint32 last_image_tick = 0U;
    uint32 last_wait_tick = 0U;
    uint32 last_motor_menu_tick = 0U;
    uint8 display_skip = 0U;
    uint8 track_lost_count = 0U;
    uint8 track_grace_count = 0U;
    int last_base_speed = 0;

    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    ips200_init(IPS200_TYPE);
    ips200_show_string(0, 304, "camera init...");

    while (mt9v03x_init())
    {
        ips200_show_string(0, 304, "camera retry...");
        system_delay_ms(500);
    }
    mt9v03x_set_reg(MT9V03X_FPS, CAMERA_FPS_TARGET);
    mt9v03x_set_exposure_time(CAMERA_EXPOSURE_TIME);
    mt9v03x_set_reg(MT9V03X_LR_OFFSET, 0);
    mt9v03x_set_reg(MT9V03X_UD_OFFSET, 0);
    mt9v03x_set_reg(MT9V03X_GAIN, CAMERA_GAIN_VALUE);
    mt9v03x_set_reg(MT9V03X_PCLK_MODE, 0);

    ips200_show_string(0, 304, "imu init...   ");
    ips200_show_string(0, 304, imu_init() ? "imu fail      " : "imu ok        ");

    image_reset();
    Init_menu();
    key_init(10);
    motor_init();
    init_encoder();
    key_state_reset();
    motor_pid_reset();

    ips200_show_string(0, 304, "bt init...    ");
    if (bluetooth_app_init()) ips200_show_string(0, 304, "bt fail       ");
    else if (bluetooth_app_baud == BLUETOOTH_APP_TARGET_BAUD)
        ips200_show_string(0, 304, "bt 115200 ok  ");
    else
        ips200_show_string(0, 304, "bt 9600 ok    ");

    pit_ms_init(TIM6_PIT, SYS_TICK_MS);
    pit_ms_init(TIM7_PIT, 5);
    Show_menu();

    while (1)
    {
        uint32 now = g_sys_tick;

        if (now - last_imu_tick >= 2U)
        {
            last_imu_tick = now;
            imu_update();
        }
        bluetooth_app_process();

        if (now - last_key_tick >= 2U)
        {
            last_key_tick = now;
            if (key_handle()) Show_menu();
        }
        if (MOTOR_PWM_TEST_ENABLE && menu_is_motor_page() &&
            now - last_motor_menu_tick >= 50U)
        {
            last_motor_menu_tick = now;
            Show_menu();
        }

        if (base_speed > 0)
        {
#if SPEED_DECISION_ENABLE
            base_speed = speed_decision_speed;
#else
            base_speed = run_base_speed;
#endif
        }

        if (base_speed > 0 && last_base_speed <= 0)
        {
            reset_run_state();
            track_grace_count = TRACK_START_GRACE_FRAMES;
            track_lost_count = 0U;
            ips200_show_string(0, 288, "RUNNING          ");
        }
        else if (base_speed <= 0 && last_base_speed > 0)
        {
            cross_state_reset();
            track_grace_count = 0U;
            track_lost_count = 0U;
        }
        last_base_speed = base_speed;

        if (mt9v03x_finish_flag &&
            (last_image_tick == 0U || now - last_image_tick >= image_ms_to_ticks(image_period_ms)))
        {
            uint32 process_start = now;
            uint8 current_threshold;

            mt9v03x_finish_flag = 0;
            if (last_image_tick != 0U)
            {
                image_frame_ms = (int)((now - last_image_tick) * SYS_TICK_MS);
                image_fps = (image_frame_ms > 0) ? 1000 / image_frame_ms : 0;
            }
            last_image_tick = now;

            memcpy(base_image, mt9v03x_image, sizeof(base_image));
            current_threshold = otsu_threshold(base_image);
            threshold = current_threshold;
            set_image_twovalues(current_threshold);
            find_base_point();
            find_boundary();
            cross_state_process();
            image_rebuild_centerline();
            image_update_steering();

            if (zebra_process())
            {
                car_stop("ZEBRA STOP      ");
            }
            else if (!(MOTOR_PWM_TEST_ENABLE && MOTOR_PWM_TEST_IGNORE_TRACK_LOST) &&
                     base_speed > 0)
            {
                if (track_grace_count > 0U)
                {
                    track_grace_count--;
                    track_lost_count = 0U;
                }
                else if (image_track_lost())
                {
                    if (track_lost_count < 255U) track_lost_count++;
                }
                else track_lost_count = 0U;

                if (track_lost_count >= TRACK_LOST_CONFIRM_FRAMES)
                {
                    car_stop("TRACK LOST STOP ");
                    track_lost_count = 0U;
                }
            }

            steering_set_image_error(track_steering_value);
#if SPEED_DECISION_ENABLE
            speed_decision_update();
#endif
            image_proc_ms = (int)((g_sys_tick - process_start) * SYS_TICK_MS);

            if (menu_is_image_page() && ++display_skip >= IMAGE_DISPLAY_SKIP_FRAMES)
            {
                display_skip = 0U;
                ips200_show_gray_image(0, 120, base_image[0], MT9V03X_W, MT9V03X_H,
                                       188, 120, current_threshold);
                draw_boundary();
                ips200_show_string(0, 256, "ERR ");
                ips200_show_float(32, 256, track_steering_value, 4, 1);
                ips200_show_string(72, 256, "CF ");
                ips200_show_int(96, 256, track_confidence, 3);
                ips200_show_string(128, 256, "W  ");
                ips200_show_int(152, 256, track_recent_width_px, 3);
                ips200_show_string(0, 272, "IMG ");
                ips200_show_int(32, 272, image_frame_ms, 3);
                ips200_show_string(60, 272, "ms ");
                ips200_show_int(84, 272, image_fps, 3);
                ips200_show_string(112, 272, "fps");
            }
            else if (!menu_is_image_page()) display_skip = 0U;
        }
        else if (!mt9v03x_finish_flag && now - last_wait_tick >= image_ms_to_ticks(image_period_ms))
        {
            last_wait_tick = now;
            if (image_wait_count < 999999) image_wait_count++;
        }
    }
}
