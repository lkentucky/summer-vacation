#include "image.h"
#include "cross.h"

#define IMAGE_CENTER_COL              (MT9V03X_W / 2)
#define IMAGE_OTSU_MIN                35U
#define IMAGE_OTSU_MAX               220U
#define IMAGE_OTSU_MAX_STEP           10U
#define IMAGE_WIDTH_FAR_DEFAULT       28
#define IMAGE_WIDTH_NEAR_DEFAULT     158
#define IMAGE_SWEEP_OFFSET             8
#define IMAGE_BLACK_SAMPLE_STEP        5
#define IMAGE_BLACK_LOST_ROWS          5
#define IMAGE_STEER_NEAR_WEIGHT     2.5f
#define IMAGE_STEER_FAR_WEIGHT      2.0f

uint8 base_image[MT9V03X_H][MT9V03X_W];
uint8 twovalues_image[MT9V03X_H][MT9V03X_W];
uint8 base_point_left = IMAGE_EDGE_MARGIN;
uint8 base_point_right = MT9V03X_W - 1U - IMAGE_EDGE_MARGIN;
uint8 search_end_line = IMAGE_SEARCH_TOP;
uint8 left_line[MT9V03X_H];
uint8 right_line[MT9V03X_H];
uint8 mid_line[MT9V03X_H];
uint8 left_line_valid[MT9V03X_H];
uint8 right_line_valid[MT9V03X_H];
uint8 mid_line_valid[MT9V03X_H];
uint8 track_width_profile[MT9V03X_H];
float mid_single_edge_width_scale = 1.0f;
int track_recent_width_px = 80;
float track_steering_value = 0.0f;
uint8 track_confidence = 0U;

static uint8 image_threshold_filtered = 128U;
static bool image_threshold_ready = false;
static float mid_output_filtered = (float)IMAGE_CENTER_COL;
static float steering_output_filtered = 0.0f;

static int image_limit_int(int value, int lower, int upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

void image_reset(void)
{
    uint16 row;

    base_point_left = IMAGE_EDGE_MARGIN;
    base_point_right = MT9V03X_W - 1U - IMAGE_EDGE_MARGIN;
    track_recent_width_px = 80;
    track_steering_value = 0.0f;
    track_confidence = 0U;
    mid_output_filtered = (float)IMAGE_CENTER_COL;
    steering_output_filtered = 0.0f;

    for (row = 0; row < MT9V03X_H; row++)
    {
        uint16 numerator = (uint16)(IMAGE_WIDTH_NEAR_DEFAULT - IMAGE_WIDTH_FAR_DEFAULT) * row;
        track_width_profile[row] = (uint8)(IMAGE_WIDTH_FAR_DEFAULT + numerator / (MT9V03X_H - 1U));
        left_line[row] = IMAGE_EDGE_MARGIN;
        right_line[row] = MT9V03X_W - 1U - IMAGE_EDGE_MARGIN;
        mid_line[row] = IMAGE_CENTER_COL;
        left_line_valid[row] = 0U;
        right_line_valid[row] = 0U;
        mid_line_valid[row] = 0U;
    }
}

uint8 otsu_threshold(uint8 image[][MT9V03X_W])
{
    uint32 histogram[256] = {0};
    uint32 total = 0;
    uint32 total_sum = 0;
    uint32 background_count = 0;
    uint32 background_sum = 0;
    float best_variance = -1.0f;
    uint8 best_threshold = image_threshold_filtered;
    uint16 row;
    uint16 col;
    uint16 level;

    /* 只统计有效赛道ROI，跳过最上方噪声和左右边缘。 */
    for (row = IMAGE_SEARCH_TOP; row < MT9V03X_H; row++)
    {
        for (col = IMAGE_EDGE_MARGIN; col < MT9V03X_W - IMAGE_EDGE_MARGIN; col++)
        {
            histogram[image[row][col]]++;
            total++;
        }
    }

    for (level = 0; level < 256; level++)
        total_sum += (uint32)level * histogram[level];

    for (level = 0; level < 256; level++)
    {
        uint32 foreground_count;
        float background_mean;
        float foreground_mean;
        float delta;
        float variance;

        background_count += histogram[level];
        background_sum += (uint32)level * histogram[level];
        if (background_count == 0U) continue;
        foreground_count = total - background_count;
        if (foreground_count == 0U) break;

        background_mean = (float)background_sum / (float)background_count;
        foreground_mean = (float)(total_sum - background_sum) / (float)foreground_count;
        delta = background_mean - foreground_mean;
        variance = (float)background_count * (float)foreground_count * delta * delta;
        if (variance > best_variance)
        {
            best_variance = variance;
            best_threshold = (uint8)level;
        }
    }

    best_threshold = (uint8)image_limit_int(best_threshold, IMAGE_OTSU_MIN, IMAGE_OTSU_MAX);
    if (!image_threshold_ready)
    {
        image_threshold_filtered = best_threshold;
        image_threshold_ready = true;
    }
    else
    {
        int delta = (int)best_threshold - (int)image_threshold_filtered;
        delta = image_limit_int(delta, -(int)IMAGE_OTSU_MAX_STEP, IMAGE_OTSU_MAX_STEP);
        image_threshold_filtered = (uint8)((int)image_threshold_filtered + delta);
    }
    return image_threshold_filtered;
}

void set_image_twovalues(uint8 threshold)
{
    uint32 global_sum = 0;
    uint32 global_count = 0;
    uint8 row;
    uint8 col;
    int global_mean;

    /* 行亮度补偿可处理上暗下亮、单侧照明等情况，又不需要大积分图缓存。 */
    for (row = IMAGE_SEARCH_TOP; row < MT9V03X_H; row += 2U)
        for (col = IMAGE_EDGE_MARGIN; col < MT9V03X_W - IMAGE_EDGE_MARGIN; col += 4U)
        {
            global_sum += base_image[row][col];
            global_count++;
        }
    global_mean = (global_count > 0U) ? (int)(global_sum / global_count) : threshold;

    for (row = 0; row < MT9V03X_H; row++)
    {
        uint32 row_sum = 0;
        uint16 row_count = 0;
        int row_mean;
        int row_threshold;

        for (col = IMAGE_EDGE_MARGIN; col < MT9V03X_W - IMAGE_EDGE_MARGIN; col += 4U)
        {
            row_sum += base_image[row][col];
            row_count++;
        }
        row_mean = (row_count > 0U) ? (int)(row_sum / row_count) : global_mean;
        row_threshold = (int)threshold + ((row_mean - global_mean) * 3) / 8;
        row_threshold = image_limit_int(row_threshold, IMAGE_OTSU_MIN, IMAGE_OTSU_MAX);

        for (col = 0; col < MT9V03X_W; col++)
            twovalues_image[row][col] = (base_image[row][col] >= row_threshold) ? 255U : 0U;
    }
}

void find_base_point(void)
{
    int seed = IMAGE_CENTER_COL;
    int longest = 0;
    int col;

    /* 参考库使用“从底部向上的最长白列”作扫线种子，
       比只看底行的白块更不容被斑马线和局部黑块带跑。 */
    for (col = MT9V03X_W / 4; col <= MT9V03X_W * 3 / 4; col += 5)
    {
        int row;
        int length = 0;
        for (row = MT9V03X_H - 1; row >= 0; row--)
        {
            if (twovalues_image[row][col] == 0U)
            {
                length = MT9V03X_H - 1 - row;
                break;
            }
        }
        if (length > longest)
        {
            longest = length;
            seed = col;
        }
    }
    base_point_left = (uint8)seed;
    base_point_right = (uint8)seed;
}

void find_boundary(void)
{
    int seed = ((int)base_point_left + (int)base_point_right) / 2;
    int previous_left;
    int previous_right;
    int lost_black_rows = 0;
    int row;

    for (row = 0; row < MT9V03X_H; row++)
    {
        left_line[row] = IMAGE_EDGE_MARGIN;
        right_line[row] = MT9V03X_W - 1U - IMAGE_EDGE_MARGIN;
        left_line_valid[row] = 0U;
        right_line_valid[row] = 0U;
        mid_line[row] = IMAGE_CENTER_COL;
        mid_line_valid[row] = 0U;
    }

    /* 底行先从种子向两边扫，扫到图像边缘便视为该侧丢线。 */
    previous_left = seed;
    while (previous_left > 2 && twovalues_image[MT9V03X_H - 1][previous_left - 1])
        previous_left--;
    previous_right = seed;
    while (previous_right < MT9V03X_W - 3 &&
           twovalues_image[MT9V03X_H - 1][previous_right + 1])
        previous_right++;

    if (twovalues_image[MT9V03X_H - 1][seed] &&
        previous_left > 2 && previous_right < MT9V03X_W - 3 &&
        previous_left < previous_right)
    {
        left_line[MT9V03X_H - 1] = (uint8)previous_left;
        left_line_valid[MT9V03X_H - 1] = 1U;
        right_line[MT9V03X_H - 1] = (uint8)previous_right;
        right_line_valid[MT9V03X_H - 1] = 1U;
    }

    /* 上一行的边界是下一行的唯一搜索先验；只有双边同时有效才更新先验。
       这是参考库比“每行全图找最近边”更稳定的关键。 */
    for (row = MT9V03X_H - 2; row >= (int)IMAGE_SWEEP_TOP; row--)
    {
        int left = previous_left + IMAGE_SWEEP_OFFSET;
        int right = previous_right - IMAGE_SWEEP_OFFSET;
        int sample;
        bool black_zone = true;

        left = image_limit_int(left, 2, MT9V03X_W - 4);
        if (!twovalues_image[row][left]) left = seed;
        while (left > 2 &&
               !(!twovalues_image[row][left - 1] && !twovalues_image[row][left - 2]))
            left--;
        if (left <= 2) left = -1;

        right = image_limit_int(right, 2, MT9V03X_W - 3);
        if (!twovalues_image[row][right]) right = seed;
        while (right < MT9V03X_W - 3 &&
               !(!twovalues_image[row][right + 1] && !twovalues_image[row][right + 2]))
            right++;
        if (right >= MT9V03X_W - 3) right = -1;

        if (left >= 0)
        {
            left_line[row] = (uint8)left;
            left_line_valid[row] = 1U;
        }
        if (right >= 0)
        {
            right_line[row] = (uint8)right;
            right_line_valid[row] = 1U;
        }
        if (left >= 0 && right >= 0 && left >= right)
        {
            left_line_valid[row] = 0U;
            right_line_valid[row] = 0U;
            left = -1;
            right = -1;
        }
        if (left >= 0 && right >= 0 && left < right)
        {
            previous_left = left;
            previous_right = right;
            lost_black_rows = 0;
            continue;
        }

        if (left < 0 && right < 0)
        {
            previous_left = seed;
            previous_right = seed;
            for (sample = MT9V03X_W / 4; sample <= MT9V03X_W * 3 / 4;
                 sample += IMAGE_BLACK_SAMPLE_STEP)
            {
                if (twovalues_image[row][sample])
                {
                    black_zone = false;
                    break;
                }
            }
            if (black_zone)
            {
                if (++lost_black_rows >= IMAGE_BLACK_LOST_ROWS) break;
            }
            else lost_black_rows = 0;  /* 开阔白区可能是十字，继续向远处扫。 */
        }
        else lost_black_rows = 0;
    }

    image_rebuild_centerline();
}

void image_rebuild_centerline(void)
{
    float width_scale = mid_single_edge_width_scale;
    int row;

    if (width_scale < 0.70f) width_scale = 0.70f;
    if (width_scale > 1.30f) width_scale = 1.30f;

    for (row = MT9V03X_H - 1; row >= (int)search_end_line; row--)
    {
        bool left_valid = left_line_valid[row] != 0U;
        bool right_valid = right_line_valid[row] != 0U;
        int center = IMAGE_CENTER_COL;
        int width = track_recent_width_px;

        mid_line_valid[row] = 0U;

        if (left_valid && right_valid && left_line[row] < right_line[row])
        {
            int measured_width = (int)right_line[row] - (int)left_line[row];
            if (measured_width >= IMAGE_TRACK_WIDTH_MIN && measured_width <= IMAGE_TRACK_WIDTH_MAX)
            {
                center = ((int)left_line[row] + (int)right_line[row]) / 2;
                if (!cross_repair_applied) track_recent_width_px = measured_width;
                track_width_profile[row] = (uint8)measured_width;
                mid_line_valid[row] = 1U;
            }
        }
        else if (!cross_repair_applied && left_valid)
        {
            center = (int)left_line[row] + (int)((float)width * width_scale * 0.5f);
            mid_line_valid[row] = 1U;
        }
        else if (!cross_repair_applied && right_valid)
        {
            center = (int)right_line[row] - (int)((float)width * width_scale * 0.5f);
            mid_line_valid[row] = 1U;
        }

        if (mid_line_valid[row])
            mid_line[row] = (uint8)image_limit_int(center, 0, MT9V03X_W - 1);
    }

}

void image_update_steering(void)
{
    int start_row = -1;
    int valid_count = 0;
    int dual_count = 0;
    float weighted_error = 0.0f;
    float weight_sum = 0.0f;
    float weight;
    float weight_step;
    int row;

    for (row = MT9V03X_H - 1; row >= (int)IMAGE_SWEEP_TOP; row--)
    {
        if (left_line_valid[row] || right_line_valid[row])
        {
            start_row = row;
            break;
        }
    }
    if (start_row <= (int)IMAGE_SWEEP_TOP)
    {
        track_steering_value = 0.0f;
        track_confidence = 0U;
        return;
    }

    weight = IMAGE_STEER_NEAR_WEIGHT;
    weight_step = (IMAGE_STEER_FAR_WEIGHT - IMAGE_STEER_NEAR_WEIGHT) /
                  (float)(start_row - (int)IMAGE_SWEEP_TOP);
    for (row = start_row; row >= (int)IMAGE_SWEEP_TOP; row--)
    {
        if (mid_line_valid[row] && (left_line_valid[row] || right_line_valid[row]))
        {
            weighted_error += ((int)mid_line[row] - IMAGE_CENTER_COL) * weight;
            weight_sum += weight;
            valid_count++;
            if (left_line_valid[row] && right_line_valid[row]) dual_count++;
        }
        weight += weight_step;
    }

    if (weight_sum > 0.0f)
    {
        float current = weighted_error / weight_sum;
        steering_output_filtered = 0.8f * current + 0.2f * steering_output_filtered;
        track_steering_value = steering_output_filtered;
    }
    else
    {
        steering_output_filtered = 0.0f;
        track_steering_value = 0.0f;
    }
    track_confidence = (uint8)image_limit_int(
        valid_count * 70 / 90 + dual_count * 30 / 90, 0, 100);
}

bool image_track_lost(void)
{
    int visible_rows = 0;
    int row;

    if (track_confidence >= 18U) return false;

    /* 十字开阔区仍有连续白色道路，不应被当作冲出赛道。 */
    for (row = 92; row <= 116; row += 4)
    {
        int run = 0;
        int max_run = 0;
        int col;
        for (col = IMAGE_EDGE_MARGIN; col < MT9V03X_W - IMAGE_EDGE_MARGIN; col++)
        {
            if (twovalues_image[row][col])
            {
                run++;
                if (run > max_run) max_run = run;
            }
            else run = 0;
        }
        if (max_run >= 28) visible_rows++;
    }
    return visible_rows < 2;
}

uint8 mid_line_weighted_average(void)
{
    uint32 sum = 0;
    uint32 weight_sum = 0;
    int row;

    for (row = 35; row <= 80; row++)
    {
        uint8 weight;
        if (!mid_line_valid[row]) continue;
        weight = (row <= 55) ? (uint8)(row - 34) : (uint8)(81 - row);
        sum += (uint32)mid_line[row] * weight;
        weight_sum += weight;
    }

    if (weight_sum > 0U)
    {
        float current = (float)sum / (float)weight_sum;
        mid_output_filtered = 0.72f * current + 0.28f * mid_output_filtered;
    }
    return (uint8)image_limit_int((int)(mid_output_filtered + 0.5f), 0, MT9V03X_W - 1);
}

void draw_boundary(void)
{
    uint16 row;

    for (row = search_end_line; row < MT9V03X_H; row++)
    {
        if (left_line_valid[row]) ips200_draw_point(left_line[row], 120 + row, RGB565_RED);
        if (right_line_valid[row]) ips200_draw_point(right_line[row], 120 + row, RGB565_BLUE);
        if (mid_line_valid[row]) ips200_draw_point(mid_line[row], 120 + row, RGB565_GREEN);
    }
}
