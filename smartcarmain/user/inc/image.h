#ifndef __IMAGE_H
#define __IMAGE_H

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "calculation.h"

#define RING_DETECTION_THRESHOLD1 3  // 环岛检测阈值，连续检测到环岛的次数超过该值才确认进入环岛
#define RING_DETECTION_THRESHOLD2 100 // 环岛确认阈值，连续检测到环岛的次数超过该值才确认在环岛内
#define RING_DETECTION_THRESHOLD3 5 // 环岛离开阈值，连续未检测到环岛的次数超过该值才确认离开环岛
#define RING_ENTRY_CONFIRM_THRESHOLD 5 // 右近处不丢线且中远处丢线后，切到 entry 拉线的连续确认帧数阈值
#define RING_RIGHT_NEAR_LOST_CONFIRM_THRESHOLD 5 // 右近处完全丢线后，开始信任左边界行驶的连续确认帧数阈值
#define RING_RIGHT_ENTRY_OFFSET 20     // 右环岛入环时，目标中线距离内圆边界的像素偏移，可实车调参
#define RING_RIGHT_IN_OFFSET    38     // 右环岛内，目标中线距离内圆边界的像素偏移，可实车调参
#define RING_RIGHT_EXIT_OFFSET  28     // 右环岛出环时，目标中线距离内圆边界的像素偏移，可实车调参
#define RING_LEFT_STRAIGHT_OFFSET_FAR  30 // 信任左边界直行时，远处中线偏移，可实车调参
#define RING_LEFT_STRAIGHT_OFFSET_NEAR 100 // 信任左边界直行时，近处中线偏移，可实车调参
#define RING_LEFT_ENTER_OFFSET_FAR     35 // entry 外圆入环线远处中线偏移，可实车调参
#define RING_LEFT_ENTER_OFFSET_NEAR    65 // entry 外圆入环线近处中线偏移，可实车调参
#define RING_OUTER_SEARCH_TOP     35   // 右环岛 entry 阶段，外圆边界搜索起始行
#define RING_OUTER_SEARCH_BOTTOM  85   // 右环岛 entry 阶段，外圆边界搜索终止行
#define RING_OUTER_MIN_LEFT_GAP   8    // 外圆候选点需要比当前左线更靠右的最小距离
#define RING_OUTER_MIN_POINTS     6    // 至少检测到多少个外圆候选点才认为有效
enum {ring_state_idle, ring_state_detecting, ring_state_wait_right_near_lost, ring_state_left_straight, ring_state_entry, ring_state_in, ring_state_confirmed};  // 环岛状态机状态枚举


extern uint8 base_image[MT9V03X_H][MT9V03X_W];    // 原始图像数组
extern uint8 twovalues_image[MT9V03X_H][MT9V03X_W];// 二值化图像数组
extern uint8 base_point_left;// 左边界基点
extern uint8 base_point_right;// 右边界基点
extern uint8 search_end_line;// 搜索赛道边界的终止行数
extern uint8 left_search_right_range;// 左边界向右搜索范围
extern uint8 left_search_left_range;// 左边界向左搜索范围
extern uint8 right_search_left_range;// 右边界向左搜索范围
extern uint8 right_search_right_range;// 右边界向右搜索范围
extern uint8 left_line[MT9V03X_H];              // 左边界数组
extern uint8 right_line[MT9V03X_H];             // 右边界数组
extern uint8 mid_line[MT9V03X_H];              // 中间线数组
extern uint8 lost_counter;                      // 丢线计数器
void find_base_point(void);
void set_image_twovalues(uint8 thr);
uint8 otsu_threshold(uint8 image[][MT9V03X_W]);//大津法求阈值
void find_boundary(void);
void draw_boundary(void);
void ring_state_process(void);

#endif  
