#ifndef __IMAGE_H
#define __IMAGE_H

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "calculation.h"


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
void find_base_point(void);
void set_image_twovalues(uint8 thr);
uint8 otsu_threshold(uint8 image[][MT9V03X_W]);//大津法求阈值
void find_boundary(void);
void draw_boundary(void);

#endif  
