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
#include "cross.h"
#include "IMU.h"
#include <string.h>
#include <stdbool.h>
// **************************** 代码区域 ****************************
#define IPS200_TYPE (IPS200_TYPE_SPI)
#define CAMERA_FPS_TARGET        100
#define CAMERA_EXPOSURE_TIME      200
#define CAMERA_GAIN_VALUE         45
#define IMAGE_PERIOD_MS_DEFAULT   20
#define IMAGE_PERIOD_MS_MIN       SYS_TICK_MS
#define IMAGE_PERIOD_MS_MAX       50
#define IMAGE_DISPLAY_SKIP_FRAMES  5
#define IMAGE_MENU_CROSS_VALUE_X   150       // 图像菜单中cross数值的横坐标
#define IMAGE_MENU_CROSS_VALUE_Y   (5 * 16)  // cross是图像菜单第6项，对应第5行
#define IMAGE_MENU_ZEBRA_VALUE_X   150       // 图像菜单中斑马线数据的横坐标
#define IMAGE_MENU_ZEBRA_COUNT_Y   (6 * 16)  // zebra_n是图像菜单第7项，对应第6行
#define IMAGE_MENU_ZEBRA_JUMP_Y    (7 * 16)  // zebra_jump是图像菜单第8项，对应第7行
#define IMAGE_MENU_ZEBRA_ROWS_Y    (8 * 16)  // zebra_rows是图像菜单第9项，对应第8行
#define IMAGE_MENU_ZEBRA_STATE_Y   (9 * 16)  // zebra_state是图像菜单第10项，对应第9行
#define ZEBRA_DETECT_ROW            50       // 在二值图第50行检测斑马线
#define ZEBRA_DETECT_ROW_START      48       // 多行检测起始行
#define ZEBRA_DETECT_ROW_END        52       // 多行检测结束行，共检查48~52五行
#define ZEBRA_PATTERN_REQUIRED       8       // 每行至少找到8次“白黑黑”才算该行满足条件
#define ZEBRA_MATCH_ROW_REQUIRED     3       // 五行中至少三行满足，当前帧才算斑马线候选
#define ZEBRA_CONFIRM_FRAMES         3       // 候选连续出现三帧后，才正式累计一次斑马线
#define ZEBRA_RELEASE_FRAMES         5       // 通过后连续五帧无候选，才允许检测下一条
#define ZEBRA_STOP_COUNT             2       // 累计检测到第2条斑马线后停车
#define ZEBRA_STATE_SEARCH           0       // 状态0：等待斑马线候选
#define ZEBRA_STATE_CONFIRM          1       // 状态1：连续帧确认斑马线
#define ZEBRA_STATE_PASSING          2       // 状态2：本条已计数，等待完全离开
#define STEER_CENTER_COL      (MT9V03X_W / 2)
#define STEER_NEAR_ROW_START  60
#define STEER_NEAR_ROW_END    65
#define STEER_FAR_ROW_START   35
#define STEER_FAR_ROW_END     40
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


int image_period_ms = IMAGE_PERIOD_MS_DEFAULT;  // 图像处理最小间隔，单位ms；设小可降低取帧延迟
int image_frame_ms = 0;                         // 实际两次处理之间的间隔，单位ms
int image_proc_ms = 0;                          // 一次图像处理从开始到输出误差的耗时，单位ms
int image_fps = 0;                              // 实际处理帧率，fps
int image_wait_count = 0;                       // 检查时摄像头还没给新帧的次数，持续增加代表摄像头FPS低于检查节拍
int zebra_cross_count = 0;                      // 本次运行已经通过的斑马线数量，达到2后停车
int zebra_transition_count = 0;                 // 当前二值图第50行找到的“白黑黑”次数，用于菜单观察
static bool zebra_detect_latched = false;       // 当前斑马线是否已计数，防止同一条斑马线被连续多帧重复累计
static uint8 zebra_release_frame_count = 0;     // 状态2中斑马线候选连续消失的帧数

int zebra_match_row_count = 0;                  // 48~52行中“白黑黑”次数达到6的行数
int zebra_state = ZEBRA_STATE_SEARCH;           // 斑马线状态机：0等待、1确认、2通过
static uint8 zebra_confirm_frame_count = 0;     // 状态1中连续满足多行条件的图像帧数

static uint32 image_ms_to_ticks(int ms)
{
  if (ms < IMAGE_PERIOD_MS_MIN) ms = IMAGE_PERIOD_MS_MIN;
  if (ms > IMAGE_PERIOD_MS_MAX) ms = IMAGE_PERIOD_MS_MAX;
  image_period_ms = ms;
  return (uint32)((ms + SYS_TICK_MS - 1) / SYS_TICK_MS);
}

static uint32 image_ticks_to_ms(uint32 ticks)
{
  return ticks * SYS_TICK_MS;
}


/// 根据指定行范围计算中线误差平均值，返回值为中线误差，单位像素
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

// 从指定行的图像中心分别向左、向右搜索“白、黑、黑”像素序列。
// 只有白色后面至少连续两个黑色像素才计一次，可滤掉单个黑色噪点。
static int zebra_count_row_patterns(uint8 row)
{
  int pattern_count = 0;                    // 本行左右两侧累计的“白黑黑”次数
  uint16 center_col = MT9V03X_W / 2;        // 图像中心列，左右搜索都从这里开始

  // 从中心向左搜索：col+1是内侧白点，col和col-1是向外连续的两个黑点。
  for (int16 col = (int16)center_col - 1; col >= 1; col--) {
    if (twovalues_image[row][col + 1] == 255 &&
        twovalues_image[row][col] == 0 &&
        twovalues_image[row][col - 1] == 0) {
      pattern_count++;
    }
  }

  // 从中心向右搜索：col-1是内侧白点，col和col+1是向外连续的两个黑点。
  for (uint16 col = center_col; col < MT9V03X_W - 1; col++) {
    if (twovalues_image[row][col - 1] == 255 &&
        twovalues_image[row][col] == 0 &&
        twovalues_image[row][col + 1] == 0) {
      pattern_count++;
    }
  }

  return pattern_count;
}

// 检查48~52行，返回其中“白黑黑”次数达到6的行数。
// 多行同时满足可以排除普通赛道中只影响一两行的边缘噪点。
static int zebra_count_matching_rows(void)
{
  int matching_rows = 0;  // 当前帧满足“白黑黑”次数条件的行数

  for (uint8 row = ZEBRA_DETECT_ROW_START; row <= ZEBRA_DETECT_ROW_END; row++) {
    if (zebra_count_row_patterns(row) >= ZEBRA_PATTERN_REQUIRED) {
      matching_rows++;
    }
  }

  return matching_rows;
}

// 三状态检测：0等待候选、1连续帧确认、2本条已计数并等待离开。
// 返回true表示刚确认到第2条斑马线，主循环应立即执行停车保护。
static bool zebra_state_process(void)
{
  bool zebra_candidate;  // 当前帧是否有至少三行满足跳变条件

  zebra_transition_count = zebra_count_row_patterns(ZEBRA_DETECT_ROW);
  zebra_match_row_count = zebra_count_matching_rows();
  zebra_candidate = (zebra_match_row_count >= ZEBRA_MATCH_ROW_REQUIRED);

  // 停车时保留跳变次数和匹配行数供观察，但状态机不累计。
  if (base_speed <= 0) {
    zebra_detect_latched = false;
    zebra_state = ZEBRA_STATE_SEARCH;
    zebra_confirm_frame_count = 0;
    zebra_release_frame_count = 0;
    return false;
  }

  // 状态2：本条斑马线已经计数，候选连续消失五帧后才回到等待状态。
  if (zebra_detect_latched) {
    zebra_state = ZEBRA_STATE_PASSING;
    if (zebra_candidate) {
      zebra_release_frame_count = 0;
    } else {
      if (zebra_release_frame_count < ZEBRA_RELEASE_FRAMES) {
        zebra_release_frame_count++;
      }
      if (zebra_release_frame_count >= ZEBRA_RELEASE_FRAMES) {
        zebra_detect_latched = false;
        zebra_state = ZEBRA_STATE_SEARCH;
        zebra_release_frame_count = 0;
      }
    }
    return false;
  }

  // 状态0：普通赛道或候选中断时，清空确认帧数重新等待。
  if (!zebra_candidate) {
    zebra_state = ZEBRA_STATE_SEARCH;
    zebra_confirm_frame_count = 0;
    return false;
  }

  // 状态1：五行中至少三行满足，并且必须连续保持三帧。
  zebra_state = ZEBRA_STATE_CONFIRM;
  if (zebra_confirm_frame_count < ZEBRA_CONFIRM_FRAMES) {
    zebra_confirm_frame_count++;
  }
  if (zebra_confirm_frame_count < ZEBRA_CONFIRM_FRAMES) {
    return false;
  }

  // 三帧确认完成后只累计一次，随后进入状态2等待这条斑马线离开。
  zebra_detect_latched = true;
  zebra_state = ZEBRA_STATE_PASSING;
  zebra_confirm_frame_count = 0;
  zebra_release_frame_count = 0;
  if (zebra_cross_count < ZEBRA_STOP_COUNT) {
    zebra_cross_count++;
  }

  return (zebra_cross_count >= ZEBRA_STOP_COUNT);
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
  mt9v03x_set_reg(MT9V03X_FPS, CAMERA_FPS_TARGET);  // 提高摄像头目标帧率，实际值受曝光和分辨率限制
  mt9v03x_set_exposure_time(CAMERA_EXPOSURE_TIME);    // 降低曝光上限有助于提高帧率
  mt9v03x_set_reg(MT9V03X_LR_OFFSET, 0);  // 设置摄像头左右偏移量
  mt9v03x_set_reg(MT9V03X_UD_OFFSET, 0);  // 设置摄像头上下偏移量
  mt9v03x_set_reg(MT9V03X_GAIN, CAMERA_GAIN_VALUE); // 曝光降低后适当提高增益
  mt9v03x_set_reg(MT9V03X_PCLK_MODE, 0);  // 设置摄像头像素时钟模式

  ips200_show_string(0, 304, "imu init...   ");
  if (imu_init()) {
    ips200_show_string(0, 304, "imu fail      ");
  } else {
    ips200_show_string(0, 304, "imu ok        ");
  }

  Init_menu();   // 初始化菜单数据
  key_init(10);  // 初始化按键扫描，10ms周期
   
  motor_init();  // 初始化电机控制引脚和PWM输出
  init_encoder();  // 初始化编码器

  key_state_reset();   // 复位按键状态（热复位兼容）
  motor_pid_reset();   // 复位PID积分（热复位兼容）
  pit_ms_init(TIM6_PIT, 2);   // TIM6: 速度PID每2ms，转向环在中断内分频为10ms。

  Show_menu();  // 首次显示菜单

  enum { STEP_IDLE, STEP_PROCESS, STEP_BOUNDARY,STEP_RING ,STEP_STEER, STEP_DISPLAY };

  while (1) {
    static uint32 last_key_tick = 0;
    static uint32 next_image_tick = 0;
    static uint32 last_image_tick = 0;
    static uint32 image_process_start_tick = 0;
    static uint8 step = STEP_IDLE;
    static uint8 image_display_skip = 0;
    static uint8 current_threshold = 230;
    static int last_base_speed = 0;
    static uint8 track_lost_frame_count = 0;
    static uint8 track_start_grace_count = 0;
    static uint32 last_imu_tick = 0;
    static uint32 last_motor_test_menu_tick = 0; // 电机菜单中编码器数据的最近刷新时刻。

    if (g_sys_tick - last_imu_tick >= 2) {
      last_imu_tick = g_sys_tick;
      imu_update();
    }

    if (g_sys_tick - last_key_tick >= 2) {
      last_key_tick = g_sys_tick;
      if (key_handle())
        Show_menu();
    }

    if (MOTOR_PWM_TEST_ENABLE && menu_is_motor_page() &&
        g_sys_tick - last_motor_test_menu_tick >= 50) {
      last_motor_test_menu_tick = g_sys_tick;
      Show_menu(); // 每100ms刷新一次累计计数和换算速度，避免只在按键时看到旧值。
    }

    if (base_speed > 0) {
#if SPEED_DECISION_ENABLE
      // 启用后使用速度决策输出；总开关为0时仍保持原来的固定巡线速度。
      base_speed = speed_decision_speed;
#else
      base_speed = run_base_speed;  // 运行中允许通过菜单实时调整巡线速度
#endif
    }

    if (base_speed > 0 && last_base_speed <= 0) {
      track_start_grace_count = TRACK_START_GRACE_FRAMES;
      track_lost_frame_count = 0;
      zebra_cross_count = 0;          // 每次重新发车都从第1条斑马线开始计数
      zebra_transition_count = 0;     // 清除上次停车时保留的跳变显示值
      zebra_detect_latched = false;   // 清除上次运行留下的斑马线锁存状态
      zebra_match_row_count = 0;      // 清除上次停车时保留的匹配行数
      zebra_state = ZEBRA_STATE_SEARCH; // 状态机回到等待斑马线状态
      zebra_confirm_frame_count = 0;  // 清除连续确认帧数
      zebra_release_frame_count = 0;  // 清除斑马线离开帧计数
      ips200_show_string(0, 288, "RUNNING          ");
    } else if (base_speed <= 0 && last_base_speed > 0) {
      track_start_grace_count = 0;
      track_lost_frame_count = 0;
      cross_state_reset();
    } else if (base_speed <= 0) {
      track_start_grace_count = 0;
      track_lost_frame_count = 0;
    }
    last_base_speed = base_speed;

    switch (step) {
    case STEP_IDLE:
      // 低延迟取帧：主循环持续检查 finish_flag，有新帧且满足最小处理间隔就立即处理。
      // image_period_ms 现在表示最小处理间隔；设得小一些可以减少取帧延迟，实际fps仍由摄像头决定。
      if (mt9v03x_finish_flag &&
          (last_image_tick == 0 ||
           (int32)(g_sys_tick - last_image_tick) >= (int32)image_ms_to_ticks(image_period_ms))) {
        mt9v03x_finish_flag = 0;
        if (last_image_tick != 0) {
          image_frame_ms = (int)image_ticks_to_ms(g_sys_tick - last_image_tick);
          image_fps = (image_frame_ms > 0) ? (1000 / image_frame_ms) : 0;
        }
        last_image_tick = g_sys_tick;
        image_process_start_tick = g_sys_tick;
        next_image_tick = g_sys_tick + image_ms_to_ticks(image_period_ms);
        step = STEP_PROCESS;
      } else if ((int32)(g_sys_tick - next_image_tick) >= 0) {
        next_image_tick = g_sys_tick + image_ms_to_ticks(image_period_ms);
        if (!mt9v03x_finish_flag && image_wait_count < 999999) {
          image_wait_count++;
        }
      }
      break;
    case STEP_PROCESS:
      memcpy(base_image, mt9v03x_image, sizeof(base_image));
      current_threshold = otsu_threshold(base_image);
      set_image_twovalues(current_threshold);
      if (zebra_state_process()) {
        car_stop_protect();
        ips200_show_string(0, 288, "ZEBRA STOP      ");
        step = STEP_IDLE;
        break;
      }
      if (menu_is_image_page()) {
        // 图像菜单不整体重画，这里实时刷新斑马线调试数据。
        ips200_show_int(IMAGE_MENU_ZEBRA_VALUE_X, IMAGE_MENU_ZEBRA_COUNT_Y,
                        zebra_cross_count, 3);
        ips200_show_int(IMAGE_MENU_ZEBRA_VALUE_X, IMAGE_MENU_ZEBRA_JUMP_Y,
                        zebra_transition_count, 3);
        ips200_show_int(IMAGE_MENU_ZEBRA_VALUE_X, IMAGE_MENU_ZEBRA_ROWS_Y,
                        zebra_match_row_count, 3);
        ips200_show_int(IMAGE_MENU_ZEBRA_VALUE_X, IMAGE_MENU_ZEBRA_STATE_Y,
                        zebra_state, 3);
      }
      find_base_point();
      step = STEP_BOUNDARY;
      break;
    case STEP_BOUNDARY:
      find_boundary();
      cross_state_process();  // 普通边线搜索后执行十字检测与补线
      if (menu_is_image_page()) {
        // 菜单整体只在按键动作时重画，这里单独实时刷新cross状态，
        // 否则短暂的exit_confirm状态会被屏幕上的旧数值掩盖。
        ips200_show_int(IMAGE_MENU_CROSS_VALUE_X, IMAGE_MENU_CROSS_VALUE_Y,
                        cross_state, 3);
      }
      if (MOTOR_PWM_TEST_ENABLE && MOTOR_PWM_TEST_IGNORE_TRACK_LOST) {
        // 固定PWM台架测试没有赛道图像，跳过丢线停车；双击K4仍可立即停止电机。
        step = STEP_STEER;
      } else if (base_speed > 0) {
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
          step = STEP_STEER;
        }
      } else {
        step = STEP_STEER;
      }
      break;
    // case STEP_RING:
    //   ring_state_process();  // Handle ring detection logic here
    //   step = STEP_STEER;
    //   break;
    case STEP_STEER:
      // 主反馈是多行中线加权偏差；远点偏差用于生成低通后的预瞄前馈。
      // 加权偏差不等于单独近点，控制器内使用“远点-加权偏差”而非几何远近点斜率。
      steering_set_image_error((int16)mid_line_weighted_average() - STEER_CENTER_COL,
                               get_mid_error_average(STEER_FAR_ROW_START, STEER_FAR_ROW_END),
                               (float)image_frame_ms * 0.001f);
#if SPEED_DECISION_ENABLE
      speed_decision_update();  // 每个新图像帧更新一次直道/弯道状态和目标速度
#endif
      image_proc_ms = (int)image_ticks_to_ms(g_sys_tick - image_process_start_tick);
      if (menu_is_image_page()) {
        if (++image_display_skip >= IMAGE_DISPLAY_SKIP_FRAMES) {
          image_display_skip = 0;
          step = STEP_DISPLAY;
        } else {
          step = STEP_IDLE;
        }
      } else {
        image_display_skip = 0;
        step = STEP_IDLE;
      }
      break;
    case STEP_DISPLAY:
      // 图像菜单顶部保留参数，图像显示在y=100以下；离开菜单后不会再刷新屏幕图像。
      ips200_show_gray_image(0, 120, base_image[0], MT9V03X_W, MT9V03X_H,
                             188, 120, current_threshold);
      draw_boundary();
      ips200_show_string(0, 256, "MID " );
      ips200_show_int(32, 256, (int)mid_line_weighted_average(), 3);
      ips200_show_string(0, 272, "IMG " );
      ips200_show_int(32, 272, image_frame_ms, 3);
      ips200_show_string(58, 272, "ms " );
      ips200_show_int(82, 272, image_fps, 3);
      ips200_show_string(108, 272, "fps " );
      ips200_show_int(142, 272, image_proc_ms, 2);
      ips200_show_string(160, 272, "ms");
      step = STEP_IDLE;
      break;
    }
  }
}
// **************************** 代码区域 ****************************
