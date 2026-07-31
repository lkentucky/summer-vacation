#ifndef __CROSS_H_
#define __CROSS_H_

#include "zf_common_headfile.h"

/*
 * 简化后的十字状态：
 * idle         ：普通巡线，等待十字入口。
 * detecting    ：入口已经锁存，固定端点补线7帧。
 */
enum
{
    cross_state_idle = 0,       // 0：普通巡线
    cross_state_detecting       // 1：固定补线10帧
};

extern uint8 cross_state;       // 当前状态，可在图像菜单cross参数中观察

// 每个图像帧在find_boundary()之后调用一次。
void cross_state_process(void);

// 停车时解除锁存、清除远点缓存并回到idle。
void cross_state_reset(void);

#endif
