/*********************************************************************************************************************
 * MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK
 * 接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 MM32F327X-G8P 开源库的一部分
 *
 * MM32F327X-G8P 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即
 * GNU通用公共许可证）的条款 即 GPL 的第3版（即
 * GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt
 * 文件中 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.37
 * 适用平台          MM32F327X_G8P
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-08-10        Teternal            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "image.h"
#include <string.h>
#include <stdbool.h>
// **************************** 代码区域 ****************************
#define IPS200_TYPE (IPS200_TYPE_SPI)
#define STEER_CENTER_COL      (MT9V03X_W / 2)
#define STEER_NEAR_ROW_START  100
#define STEER_NEAR_ROW_END    119
#define STEER_FAR_ROW_START   45
#define STEER_FAR_ROW_END     65
#define STEER_MAX_RATIO       (0.85f)
#define STEER_MAX_STEP        (15.0f)
#define STEER_D_GAIN          (0.05f)
#define TRACK_LOST_ROW_START       92
#define TRACK_LOST_ROW_END         115
#define TRACK_LOST_COUNT_TH        23
#define TRACK_LOST_CONFIRM_FRAMES  4
#define TRACK_START_GRACE_FRAMES   12
#define TRACK_WIDTH_MIN            20
#define TRACK_WIDTH_MAX            175
#define TRACK_EDGE_MARGIN          3
// 十字/环岛入口处近处经常是一大片白色开阔区域，左右边界会同时丢线。
// 只要原始二值图还能看到足够宽的白色赛道，就不要把“开阔区域”当成冲出赛道。
#define TRACK_ROAD_WHITE_RUN_MIN       28
#define TRACK_ROAD_WHITE_COUNT_MIN     45
#define TRACK_CENTER_WHITE_HALF_WIDTH  8
#define TRACK_CENTER_WHITE_COUNT_TH    6


static int16 get_mid_error_average(uint8 start_row, uint8 end_row)
{
  int32 sum = 0;
  uint8 count = 0;

  if (start_row >= MT9V03X_H) start_row = MT9V03X_H - 1;
  if (end_row >= MT9V03X_H) end_row = MT9V03X_H - 1;
  if (start_row > end_row) {
    uint8 temp = start_row;
    start_row = end_row;
    end_row = temp;
  }

  for (uint8 i = start_row; i <= end_row; i++) {
    sum += mid_line[i];
    count++;
  }

  if (count == 0) return 0;
  return (int16)(sum / count) - STEER_CENTER_COL;
}

static float limit_float(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

static bool track_row_has_visible_road(uint8 row)
{
  uint8 white_count = 0;
  uint8 white_run = 0;
  uint8 max_white_run = 0;
  uint8 center_white_count = 0;
  uint8 center_left = STEER_CENTER_COL - TRACK_CENTER_WHITE_HALF_WIDTH;
  uint8 center_right = STEER_CENTER_COL + TRACK_CENTER_WHITE_HALF_WIDTH;

  for (uint8 j = 2; j < MT9V03X_W - 2; j++) {
    if (twovalues_image[row][j] == 255) {
      white_count++;
      white_run++;
      if (white_run > max_white_run) {
        max_white_run = white_run;
      }
      if (j >= center_left && j <= center_right) {
        center_white_count++;
      }
    } else {
      white_run = 0;
    }
  }

  return (max_white_run >= TRACK_ROAD_WHITE_RUN_MIN &&
          (white_count >= TRACK_ROAD_WHITE_COUNT_MIN ||
           center_white_count >= TRACK_CENTER_WHITE_COUNT_TH));
}

static bool track_lost_detect(void)
{
  uint8 bad_count = 0;

  for (uint8 i = TRACK_LOST_ROW_START; i <= TRACK_LOST_ROW_END; i++) {
    bool left_lost = (left_line[i] <= 2);
    bool right_lost = (right_line[i] >= MT9V03X_W - 3);
    bool both_lost = left_lost && right_lost;
    bool mid_edge = (mid_line[i] <= TRACK_EDGE_MARGIN || mid_line[i] >= MT9V03X_W - 1 - TRACK_EDGE_MARGIN);
    bool road_visible = track_row_has_visible_road(i);
    bool width_bad = false;

    // 十字和环岛会出现“左右同时丢线/宽度过大/中线贴边”，但图像下方仍有大块白色赛道。
    // 这种情况是特殊赛道开阔区，不是冲出赛道，直接跳过本行。
    if (road_visible && (both_lost || mid_edge)) {
      continue;
    }

    // 单边丢线在弯道/环岛中很常见，不直接算冲出赛道。
    // 只有左右都有效但宽度极不合理，或者左右同时丢线/中线贴边，才计入坏行。
    if (!left_lost && !right_lost) {
      if (right_line[i] > left_line[i]) {
        uint8 width = right_line[i] - left_line[i];
        width_bad = (width < TRACK_WIDTH_MIN || width > TRACK_WIDTH_MAX);

        // 宽度过大通常对应十字/环岛开阔区；只要白色赛道仍然可见，就不算坏行。
        if (road_visible && width > TRACK_WIDTH_MAX) {
          width_bad = false;
        }
      } else {
        width_bad = true;
      }
    }

    if (both_lost || mid_edge || width_bad) {
      bad_count++;
    }
  }

  return (bad_count >= TRACK_LOST_COUNT_TH);
}


static void car_stop_protect(void)
{
  base_speed = 0;
  target_speedl = 0.0f;
  target_speedr = 0.0f;
  motor_pid_reset();
}

int main(void) {
  clock_init(SYSTEM_CLOCK_120M);  // 必须先初始化时钟
  debug_init();                   // 初始化 Debug UART

  ips200_init(IPS200_TYPE);  // 先初始化屏幕
  ips200_show_string(0, 304, "camera init...");

  while (mt9v03x_init()) {  // 初始化摄像头，失败则重试
    ips200_show_string(0, 304, "camera retry...");
    system_delay_ms(500);
  }
  ips200_show_string(0, 304, "camera ok     ");
  mt9v03x_set_exposure_time(512);         // 设置摄像头曝光时间
  mt9v03x_set_reg(MT9V03X_LR_OFFSET, 0);  // 设置摄像头左右偏移量
  mt9v03x_set_reg(MT9V03X_UD_OFFSET, 0);  // 设置摄像头上下偏移量
  mt9v03x_set_reg(MT9V03X_GAIN, 32);      // 设置摄像头图像增益
  mt9v03x_set_reg(MT9V03X_PCLK_MODE, 0);  // 设置摄像头像素时钟模式

  Init_menu();   // 初始化菜单数据
  key_init(10);  // 初始化按键扫描，10ms周期
   
  motor_init();  // 初始化电机控制引脚和PWM输出
  init_encoder();  // 初始化编码器

  key_state_reset();   // 复位按键状态（热复位兼容）
  motor_pid_reset();   // 复位PID积分（热复位兼容）
  pit_ms_init(TIM6_PIT, 5);   // TIM6: PID控速，5ms

  Show_menu();  // 首次显示菜单

  enum { STEP_IDLE, STEP_PROCESS, STEP_BOUNDARY,STEP_RING ,STEP_STEER, STEP_DISPLAY };

  while (1) {
    static uint32 last_key_tick = 0;
    static uint32 last_display_tick = 0;
    static uint8 step = STEP_IDLE;
    static uint8 frame_skip = 9;
    static int last_base_speed = 0;
    static uint8 track_lost_frame_count = 0;
    static uint8 track_start_grace_count = 0;

    if (g_sys_tick - last_key_tick >= 2) {
      last_key_tick = g_sys_tick;
      if (key_handle())
        Show_menu();
    }

    if (base_speed > 0) {
      base_speed = run_base_speed;  // 运行中允许通过菜单实时调整巡线速度
    }

    if (base_speed > 0 && last_base_speed <= 0) {
      track_start_grace_count = TRACK_START_GRACE_FRAMES;
      track_lost_frame_count = 0;
      ips200_show_string(0, 288, "RUNNING          ");
    } else if (base_speed <= 0) {
      track_start_grace_count = 0;
      track_lost_frame_count = 0;
    }
    last_base_speed = base_speed;

    switch (step) {
    case STEP_IDLE:
      if (mt9v03x_finish_flag && g_sys_tick - last_display_tick >= 4) {
        mt9v03x_finish_flag = 0;
        step = STEP_PROCESS;
      }
      break;
    case STEP_PROCESS:
      memcpy(base_image, mt9v03x_image, sizeof(base_image));
      //uint8 thresholdb = otsu_threshold(base_image);
      set_image_twovalues(threshold);
      find_base_point();
      step = STEP_BOUNDARY;
      break;
    case STEP_BOUNDARY:
      find_boundary();
      if (base_speed > 0) {
        if (track_start_grace_count > 0) {
          track_start_grace_count--;
          track_lost_frame_count = 0;
        } else if (track_lost_detect()) {
          if (track_lost_frame_count < 255) {
            track_lost_frame_count++;
          }
        } else {
          track_lost_frame_count = 0;
        }

        if (track_lost_frame_count >= TRACK_LOST_CONFIRM_FRAMES) {
          car_stop_protect();
          track_lost_frame_count = 0;
          ips200_show_string(0, 288, "TRACK LOST STOP ");
          step = STEP_IDLE;
        } else {
          step = STEP_RING;
        }
      } else {
        step = STEP_RING;
      }
      break;
    case STEP_RING:
      ring_state_process();  // Handle ring detection logic here
      step = STEP_STEER;
      break;
    case STEP_STEER:
      {
        static int16 last_error = 0;
        static float last_steering = 0.0f;

        if (base_speed > 0) {
          int16 error_near = get_mid_error_average(STEER_NEAR_ROW_START, STEER_NEAR_ROW_END);
          int16 error_far = get_mid_error_average(STEER_FAR_ROW_START, STEER_FAR_ROW_END);
          int16 preview_error = error_far - error_near;  // 前瞻项：看前方赛道弯向
          int16 d_error = error_near - last_error;       // 真正的时间微分项：抑制摆动

          float steering = Kp_steer * error_near
                         + Kd_steer_position * preview_error
                         + Kd_steer_time * d_error;

          float max_steering = (float)base_speed * STEER_MAX_RATIO;
          steering = limit_float(steering, -max_steering, max_steering);
          steering = limit_float(steering,
                                 last_steering - STEER_MAX_STEP,
                                 last_steering + STEER_MAX_STEP);

          last_error = error_near;
          last_steering = steering;

          target_speedl = base_speed + steering;
          target_speedr = base_speed - steering;
        } else {
          last_error = 0;
          last_steering = 0.0f;
          target_speedl = 0.0f;
          target_speedr = 0.0f;
        }
      }
      step = STEP_IDLE;
      break;
    // case STEP_DISPLAY:
    //   draw_boundary();
    //   if (++frame_skip >= 10) {
    //     frame_skip = 0;
    //     ips200_show_gray_image(0, 100, mt9v03x_image[0], MT9V03X_W, MT9V03X_H,
    //                            188, 120, threshold);
    //   }
    //   printf("%.2f,%d,%.2f,%d\n", real_speedl, (int)target_speedl, real_speedr, (int)target_speedr);
    //   last_display_tick = g_sys_tick;
    //   step = STEP_IDLE;
    //   break;
    }
  }
}
// **************************** 代码区域 ****************************
