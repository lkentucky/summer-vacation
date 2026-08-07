#include "cross.h"
#include "image.h"

/*
 * 简化十字补线状态机
 *
 * 1. 40~70行左右边线同时丢失，80~99行近处双边线仍有效时，触发十字。
 * 2. 一旦进入detecting，不再因为下一帧特征消失而退回idle。
 * 3. 补线不依赖近处边线：
 *
 *             左远点 *                 * 右远点
 *                    \                 /
 *                     \               /
 *                      \             /
 *                       \           /
 *          底行固定点(5)           (图像宽度-10)
 *
 * 4. 左右远点只从十字开阔区上方寻找，并要求至少连续3行、相邻行跳变不过大。
 * 5. 每次触发只补7帧，不再判断十字出口；入口特征消失后才允许下一次触发。
 */

#define CROSS_MID_START_ROW               40  // 十字开阔区检测起始行
#define CROSS_MID_END_ROW                 70  // 十字开阔区检测结束行
#define CROSS_NEAR_START_ROW              80  // 入口阶段近处检测起始行
#define CROSS_NEAR_END_ROW                (MT9V03X_H - 21) // 当前为99行
#define CROSS_FAR_END_ROW                 (CROSS_MID_START_ROW - 1) // 远点绝不进入十字检测区

#define CROSS_MID_LOST_PERCENT            60  // 中部左右同时丢线比例
#define CROSS_NEAR_VALID_PERCENT          50  // 入口阶段近处双边线有效比例
#define CROSS_REPAIR_FRAMES               8   // 每次识别十字后固定补线帧数

#define CROSS_FAR_MIN_RUN_ROWS            3   // 远点必须来自至少3行连续边线
#define CROSS_FAR_MAX_COL_STEP            8   // 连续两行边线允许的最大横向跳
#define CROSS_FAR_CENTER_MARGIN           5   // 远点必须位于图像中心对应侧
#define CROSS_WIDTH_MIN                   20  // 左右远点允许的最小间距
#define CROSS_WIDTH_MAX                   (MT9V03X_W - 10)

#define CROSS_LEFT_BOTTOM_COL             5   // 左补线在图像底行的固定列
#define CROSS_RIGHT_BOTTOM_COL            (MT9V03X_W - 10) // 从右向左数第10个点

#define CROSS_SIDE_LEFT                   0
#define CROSS_SIDE_RIGHT                  1

typedef struct
{
    bool valid;      // 是否已经找到并缓存该侧远点
    uint8 row;       // 远点所在行
    uint8 col;       // 远点所在列
} cross_far_point_t;

uint8 cross_state = cross_state_idle;

static uint16 cross_detect_frame_count = 0; // detecting已经持续的图像帧数
static bool cross_entry_armed = true;       // 入口特征消失后才重新允许触发
static cross_far_point_t cross_left_far = {false, 0, 0};
static cross_far_point_t cross_right_far = {false, 0, 0};


// 左边线贴近图像左边缘时视为丢线。
static bool cross_left_valid(uint8 row)
{
    return (left_line[row] > 2 && left_line[row] < MT9V03X_W - 3);
}


// 右边线贴近图像右边缘时视为丢线。
static bool cross_right_valid(uint8 row)
{
    return (right_line[row] > 2 && right_line[row] < MT9V03X_W - 3);
}


static uint8 cross_count_both_lost(uint8 start_row, uint8 end_row)
{
    uint8 count = 0;

    for (uint8 row = start_row; row <= end_row; row++)
    {
        if (!cross_left_valid(row) && !cross_right_valid(row))
        {
            count++;
        }
    }
    return count;
}


// 只判断左右边线有没有丢，不再检查左右顺序和赛道宽度。
static uint8 cross_count_both_valid(uint8 start_row, uint8 end_row)
{
    uint8 count = 0;

    for (uint8 row = start_row; row <= end_row; row++)
    {
        if (cross_left_valid(row) && cross_right_valid(row))
        {
            count++;
        }
    }
    return count;
}


/*
 * 十字入口：
 * 中部出现大面积双丢线，同时近处仍能看到正常赛道。
 * 这里只负责从idle触发一次；进入detecting后不再反复检查该条件。
 */
static bool cross_entry_detect(void)
{
    const uint8 middle_rows = CROSS_MID_END_ROW - CROSS_MID_START_ROW + 1;
    const uint8 near_rows = CROSS_NEAR_END_ROW - CROSS_NEAR_START_ROW + 1;
    uint8 middle_lost =
        cross_count_both_lost(CROSS_MID_START_ROW, CROSS_MID_END_ROW);
    uint8 near_valid =
        cross_count_both_valid(CROSS_NEAR_START_ROW, CROSS_NEAR_END_ROW);

    return ((uint16)middle_lost * 100 >=
                (uint16)middle_rows * CROSS_MID_LOST_PERCENT &&
            (uint16)near_valid * 100 >=
                (uint16)near_rows * CROSS_NEAR_VALID_PERCENT);
}


static uint8 cross_get_side_col(uint8 side, uint8 row)
{
    return (side == CROSS_SIDE_LEFT) ? left_line[row] : right_line[row];
}


/*
 * 远线候选点必须满足：
 * 左远点在图像中心左侧，右远点在中心右侧；
 * 并且不能是find_boundary()放在图像边缘的丢线兜底值。
 */
static bool cross_far_candidate_valid(uint8 side, uint8 row)
{
    uint8 col = cross_get_side_col(side, row);

    if (side == CROSS_SIDE_LEFT)
    {
        return (cross_left_valid(row) &&
                col < MT9V03X_W / 2 - CROSS_FAR_CENTER_MARGIN);
    }

    return (cross_right_valid(row) &&
            col > MT9V03X_W / 2 + CROSS_FAR_CENTER_MARGIN);
}


/*
 * 从十字开阔区上方寻找连续远线。
 *
 * 从CROSS_FAR_END_ROW向上扫描，要求至少连续3行有效，
 * 且相邻行横向跳变不超过8像素。返回连续段的平均坐标，
 * 不再使用“找到第一个非边缘点就当远点”的旧逻辑。
 */
static bool cross_find_far_point(uint8 side, cross_far_point_t *point)
{
    int row;
    int last_col = -1;
    uint8 run_count = 0;
    uint16 row_sum = 0;
    uint16 col_sum = 0;

    for (row = CROSS_FAR_END_ROW; row > search_end_line; row--)
    {
        if (cross_far_candidate_valid(side, (uint8)row))
        {
            uint8 col = cross_get_side_col(side, (uint8)row);
            int col_step = (last_col < 0) ? 0 : (int)col - last_col;
            if (col_step < 0)
            {
                col_step = -col_step;
            }

            if (run_count == 0 || col_step <= CROSS_FAR_MAX_COL_STEP)
            {
                run_count++;
                row_sum += (uint8)row;
                col_sum += col;
            }
            else
            {
                run_count = 1;
                row_sum = (uint8)row;
                col_sum = col;
            }
            last_col = col;

            if (run_count >= CROSS_FAR_MIN_RUN_ROWS)
            {
                point->valid = true;
                point->row = (uint8)(row_sum / run_count);
                point->col = (uint8)(col_sum / run_count);
                return true;
            }
        }
        else
        {
            last_col = -1;
            run_count = 0;
            row_sum = 0;
            col_sum = 0;
        }
    }

    return false;
}


/*
 * 更新远点缓存。
 * 当前帧能同时找到合理的左右远点时才整体更新，防止两侧来自不同赛道分支。
 * 十字内部暂时看不到远线时继续使用进入十字时缓存的远点。
 */
static void cross_refresh_far_points(void)
{
    cross_far_point_t left_new = {false, 0, 0};
    cross_far_point_t right_new = {false, 0, 0};

    if (cross_find_far_point(CROSS_SIDE_LEFT, &left_new) &&
        cross_find_far_point(CROSS_SIDE_RIGHT, &right_new) &&
        right_new.col > left_new.col)
    {
        uint8 width = right_new.col - left_new.col;
        if (width >= CROSS_WIDTH_MIN && width <= CROSS_WIDTH_MAX)
        {
            cross_left_far = left_new;
            cross_right_far = right_new;
        }
    }
}


// 在一侧远点和图像底部固定点之间逐行做整数直线插值。
static void cross_draw_fixed_line(uint8 side,
                                  const cross_far_point_t *far_point,
                                  uint8 bottom_col)
{
    const uint8 bottom_row = MT9V03X_H - 1;
    int16 row_span;

    if (!far_point->valid || far_point->row >= bottom_row)
    {
        return;
    }

    row_span = (int16)bottom_row - far_point->row;
    for (uint8 row = far_point->row; row <= bottom_row; row++)
    {
        int16 col = (int16)far_point->col +
            ((int16)(bottom_col - far_point->col) *
             (int16)(row - far_point->row)) / row_span;

        if (side == CROSS_SIDE_LEFT)
        {
            left_line[row] = (uint8)col;
        }
        else
        {
            right_line[row] = (uint8)col;
        }
    }
}


/*
 * 简化补线：
 * 左侧固定从底行第5列拉到左远点；
 * 右侧固定从底行向左第10列拉到右远点。
 * 全程不读取近点，因此近处边线消失不会改变补线结果。
 */
static void cross_repair_simple(void)
{
    uint8 mid_start_row;

    cross_refresh_far_points();
    if (!cross_left_far.valid || !cross_right_far.valid)
    {
        return;
    }

    cross_draw_fixed_line(CROSS_SIDE_LEFT, &cross_left_far,
                          CROSS_LEFT_BOTTOM_COL);
    cross_draw_fixed_line(CROSS_SIDE_RIGHT, &cross_right_far,
                          CROSS_RIGHT_BOTTOM_COL);

    mid_start_row = (cross_left_far.row > cross_right_far.row) ?
                    cross_left_far.row : cross_right_far.row;
    for (uint8 row = mid_start_row; row < MT9V03X_H; row++)
    {
        mid_line[row] =
            (uint8)(((uint16)left_line[row] + right_line[row]) / 2);
    }
}


static void cross_clear_far_points(void)
{
    cross_left_far.valid = false;
    cross_right_far.valid = false;
}


static void cross_change_state(uint8 next_state)
{
    cross_state = next_state;
    cross_detect_frame_count = 0;
}


void cross_state_reset(void)
{
    cross_change_state(cross_state_idle);
    cross_clear_far_points();
    cross_entry_armed = true;
}


/*
 * 状态路径：
 *
 * idle --入口触发--> detecting --补满7帧--> idle
 *
 * 入口触发帧计为第1帧，之后再补6帧，总共补7帧。
 * 补满后不判断出口，直接回到idle。
 * 为防止同一个入口特征连续重复触发，必须先看到入口特征消失，
 * 才会重新允许下一次7帧补线。
 */
void cross_state_process(void)
{
    bool entry_feature = cross_entry_detect();

    switch (cross_state)
    {
        case cross_state_idle:
            cross_clear_far_points();
            if (!entry_feature)
            {
                cross_entry_armed = true;
            }
            else if (cross_entry_armed)
            {
                cross_entry_armed = false;
                cross_change_state(cross_state_detecting);
                cross_detect_frame_count = 1;
                cross_repair_simple();
            }
            break;

        case cross_state_detecting:
            cross_repair_simple();
            if (cross_detect_frame_count < 65535)
            {
                cross_detect_frame_count++;
            }
            if (cross_detect_frame_count >= CROSS_REPAIR_FRAMES)
            {
                cross_change_state(cross_state_idle);
                cross_clear_far_points();
            }
            break;

        default:
            cross_state_reset();
            break;
    }
}
