#ifndef __IMAGE_H
#define __IMAGE_H

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

/*
 * F车视觉模块。
 * 图像保持摄像头原生120x188坐标：row越小越远，col越大越靠右。
 * 主数据流按参考库设计：逐行扫边、单边按最近路宽补中线，
 * 再对近处90行中线做加权平均，直接输出像素偏差给PPDD方向环。
 */
#define IMAGE_SEARCH_TOP              20U
#define IMAGE_EDGE_MARGIN              2U
#define IMAGE_TRACK_WIDTH_MIN         18U
#define IMAGE_TRACK_WIDTH_MAX        180U
#define IMAGE_SWEEP_TOP               30U

extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 twovalues_image[MT9V03X_H][MT9V03X_W];
extern uint8 base_point_left;
extern uint8 base_point_right;
extern uint8 search_end_line;
extern uint8 left_line[MT9V03X_H];
extern uint8 right_line[MT9V03X_H];
extern uint8 mid_line[MT9V03X_H];
extern uint8 left_line_valid[MT9V03X_H];
extern uint8 right_line_valid[MT9V03X_H];
extern uint8 mid_line_valid[MT9V03X_H];
extern uint8 track_width_profile[MT9V03X_H];
extern float mid_single_edge_width_scale;
extern int track_recent_width_px;
extern float track_steering_value;
extern uint8 track_confidence;

void image_reset(void);
uint8 otsu_threshold(uint8 image[][MT9V03X_W]);
void set_image_twovalues(uint8 threshold);
void find_base_point(void);
void find_boundary(void);
void image_rebuild_centerline(void);
void image_update_steering(void);
bool image_track_lost(void);
uint8 mid_line_weighted_average(void);
void draw_boundary(void);

#endif
