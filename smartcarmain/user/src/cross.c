#include "cross.h"
#include "image.h"

#define CROSS_OPEN_FIRST_ROW          43
#define CROSS_OPEN_LAST_ROW           72
#define CROSS_NEAR_FIRST_ROW          88
#define CROSS_NEAR_LAST_ROW          116
#define CROSS_FAR_FIRST_ROW           25
#define CROSS_FAR_LAST_ROW            42
#define CROSS_CONFIRM_FRAMES           2
#define CROSS_EXIT_CONFIRM_FRAMES      3
#define CROSS_MIN_REPAIR_FRAMES        6
#define CROSS_MAX_REPAIR_FRAMES       24

typedef struct
{
    uint8 valid;
    uint8 row;
    uint8 col;
} cross_point_t;

uint8 cross_state = cross_state_idle;
bool cross_repair_applied = false;

static uint8 cross_confirm_count = 0;
static uint8 cross_exit_count = 0;
static uint8 cross_repair_count = 0;
static bool cross_armed = true;
static cross_point_t cross_left_far;
static cross_point_t cross_right_far;
static uint8 cross_bottom_center = MT9V03X_W / 2;

static bool cross_entry_feature(void)
{
    int open_lost = 0;
    int near_valid = 0;
    int far_valid = 0;
    int row;

    for (row = CROSS_OPEN_FIRST_ROW; row <= CROSS_OPEN_LAST_ROW; row++)
        if (!left_line_valid[row] && !right_line_valid[row]) open_lost++;

    for (row = CROSS_NEAR_FIRST_ROW; row <= CROSS_NEAR_LAST_ROW; row++)
        if (left_line_valid[row] && right_line_valid[row]) near_valid++;

    for (row = CROSS_FAR_FIRST_ROW; row <= CROSS_FAR_LAST_ROW; row++)
        if (left_line_valid[row] && right_line_valid[row] &&
            left_line[row] < right_line[row]) far_valid++;

    return (open_lost * 100 >=
            (CROSS_OPEN_LAST_ROW - CROSS_OPEN_FIRST_ROW + 1) * 55 &&
            near_valid * 100 >=
            (CROSS_NEAR_LAST_ROW - CROSS_NEAR_FIRST_ROW + 1) * 35 &&
            far_valid >= 3);
}

static bool cross_normal_road_feature(void)
{
    int valid = 0;
    int row;

    for (row = CROSS_OPEN_FIRST_ROW; row <= CROSS_OPEN_LAST_ROW; row++)
        if (left_line_valid[row] && right_line_valid[row] &&
            left_line[row] < right_line[row]) valid++;

    return valid * 100 >=
           (CROSS_OPEN_LAST_ROW - CROSS_OPEN_FIRST_ROW + 1) * 70;
}

static bool cross_find_far_pair(cross_point_t *left, cross_point_t *right)
{
    int run = 0;
    int row_sum = 0;
    int left_sum = 0;
    int right_sum = 0;
    int row;

    for (row = CROSS_FAR_LAST_ROW; row >= CROSS_FAR_FIRST_ROW; row--)
    {
        int width = (int)right_line[row] - (int)left_line[row];
        if (left_line_valid[row] && right_line_valid[row] &&
            width >= IMAGE_TRACK_WIDTH_MIN && width <= IMAGE_TRACK_WIDTH_MAX)
        {
            row_sum += row;
            left_sum += left_line[row];
            right_sum += right_line[row];
            run++;
            if (run >= 4)
            {
                left->valid = 1U;
                right->valid = 1U;
                left->row = right->row = (uint8)(row_sum / run);
                left->col = (uint8)(left_sum / run);
                right->col = (uint8)(right_sum / run);
                return true;
            }
        }
        else
        {
            run = 0;
            row_sum = 0;
            left_sum = 0;
            right_sum = 0;
        }
    }
    return false;
}

static void cross_update_bottom_center(void)
{
    int sum = 0;
    int count = 0;
    int row;

    for (row = 96; row <= 116; row++)
    {
        if (mid_line_valid[row])
        {
            sum += mid_line[row];
            count++;
        }
    }
    if (count > 0) cross_bottom_center = (uint8)(sum / count);
}

static void cross_draw_side(bool left_side, cross_point_t point, int bottom_col)
{
    int bottom_row = MT9V03X_H - 1;
    int span;
    int row;

    if (!point.valid || point.row >= bottom_row) return;
    span = bottom_row - point.row;
    for (row = point.row; row <= bottom_row; row++)
    {
        int col = point.col +
                  (bottom_col - (int)point.col) * (row - (int)point.row) / span;
        if (col < IMAGE_EDGE_MARGIN) col = IMAGE_EDGE_MARGIN;
        if (col > MT9V03X_W - 1 - IMAGE_EDGE_MARGIN)
            col = MT9V03X_W - 1 - IMAGE_EDGE_MARGIN;

        if (left_side)
        {
            left_line[row] = (uint8)col;
            left_line_valid[row] = 1U;
        }
        else
        {
            right_line[row] = (uint8)col;
            right_line_valid[row] = 1U;
        }
    }
}

static bool cross_apply_repair(void)
{
    cross_point_t new_left = {0U, 0U, 0U};
    cross_point_t new_right = {0U, 0U, 0U};
    int bottom_width;
    int bottom_left;
    int bottom_right;

    if (cross_find_far_pair(&new_left, &new_right))
    {
        cross_left_far = new_left;
        cross_right_far = new_right;
    }
    if (!cross_left_far.valid || !cross_right_far.valid) return false;

    cross_update_bottom_center();
    bottom_width = track_width_profile[MT9V03X_H - 1];
    if (bottom_width < 80) bottom_width = 80;
    if (bottom_width > MT9V03X_W - 12) bottom_width = MT9V03X_W - 12;
    bottom_left = (int)cross_bottom_center - bottom_width / 2;
    bottom_right = (int)cross_bottom_center + bottom_width / 2;

    cross_draw_side(true, cross_left_far, bottom_left);
    cross_draw_side(false, cross_right_far, bottom_right);
    cross_repair_applied = true;
    image_rebuild_centerline();
    return true;
}

void cross_state_reset(void)
{
    cross_state = cross_state_idle;
    cross_confirm_count = 0U;
    cross_exit_count = 0U;
    cross_repair_count = 0U;
    cross_armed = true;
    cross_left_far.valid = 0U;
    cross_right_far.valid = 0U;
    cross_bottom_center = MT9V03X_W / 2;
    cross_repair_applied = false;
}

void cross_state_process(void)
{
    bool entry = cross_entry_feature();
    bool normal_road = cross_normal_road_feature();

    cross_repair_applied = false;

    switch (cross_state)
    {
        case cross_state_idle:
            if (!entry)
            {
                cross_armed = true;
                cross_confirm_count = 0U;
            }
            else if (cross_armed)
            {
                cross_state = cross_state_confirm;
                cross_confirm_count = 1U;
            }
            break;

        case cross_state_confirm:
            if (!entry)
            {
                cross_state = cross_state_idle;
                cross_confirm_count = 0U;
            }
            else if (++cross_confirm_count >= CROSS_CONFIRM_FRAMES)
            {
                cross_point_t left = {0U, 0U, 0U};
                cross_point_t right = {0U, 0U, 0U};
                if (cross_find_far_pair(&left, &right))
                {
                    cross_left_far = left;
                    cross_right_far = right;
                    cross_update_bottom_center();
                    cross_state = cross_state_repair;
                    cross_armed = false;
                    cross_repair_count = 0U;
                    cross_exit_count = 0U;
                    cross_apply_repair();
                }
            }
            break;

        case cross_state_repair:
            cross_repair_count++;
            if (normal_road && cross_repair_count >= CROSS_MIN_REPAIR_FRAMES)
                cross_exit_count++;
            else
                cross_exit_count = 0U;

            if (cross_exit_count >= CROSS_EXIT_CONFIRM_FRAMES ||
                cross_repair_count >= CROSS_MAX_REPAIR_FRAMES)
            {
                cross_state = cross_state_idle;
                cross_left_far.valid = 0U;
                cross_right_far.valid = 0U;
                cross_confirm_count = 0U;
                cross_exit_count = 0U;
            }
            else
            {
                cross_apply_repair();
            }
            break;

        default:
            cross_state_reset();
            break;
    }
}
