#ifndef __CROSS_H_
#define __CROSS_H_

#include "zf_common_headfile.h"

enum
{
    cross_state_idle = 0,
    cross_state_confirm,
    cross_state_repair
};

extern uint8 cross_state;
extern bool cross_repair_applied;

/* 每个图像帧在find_boundary()之后调用。 */
void cross_state_process(void);
void cross_state_reset(void);

#endif
