#include "image.h"


uint8 base_image[MT9V03X_H][MT9V03X_W];
uint8 twovalues_image[MT9V03X_H][MT9V03X_W];
uint8 base_point_left;
uint8 base_point_right;
uint8 search_end_line=30;  // 搜索赛道边界的终止行数
uint8 left_search_right_range=10;  // 左边界向右搜索范围
uint8 left_search_left_range=10;   // 左边界向左搜索范围
uint8 right_search_left_range=10;  // 右边界向左搜索范围
uint8 right_search_right_range=10;  // 右边界向右搜索范围
uint8 left_line[MT9V03X_H];  // 左边界数组
uint8 right_line[MT9V03X_H];  // 右边界数组
uint8 mid_line[MT9V03X_H];   // 中间线数组
uint8 lost_counter;          // 丢线计数器
static uint8 ring_left_straight_offset_table[MT9V03X_H]; // 信左边界直行时，每行左线到中线的偏移
static uint8 ring_left_enter_offset_table[MT9V03X_H];    // entry 入环线时，每行左线到中线的偏移
static uint8 ring_offset_table_inited = 0;

static bool enter_detect_ring(void);
static bool exit_detect_ring(void);
static bool right_entry_ready_detect(void);
static bool right_near_full_lost_detect(void);
static void enter_ring_handle(void);
static void in_ring_handle(void);
static void exit_ring_handle(void);
static void left_boundary_straight_handle(void);
static void left_boundary_array_offset_handle(const uint8 offset_table[]);
static void ring_offset_table_init(void);
static uint8 ring_calc_row_offset(uint8 row, uint8 far_offset, uint8 near_offset);
static void ring_outer_entry_line_handle(void);
static bool find_right_ring_outer_point(uint8 *outer_row, uint8 *outer_col);
static void lost_location_detection(uint8 l_or_r, uint8 start_row, uint8 end_row);
static void ring_right_inner_line_handle(int16 offset);

//大津法(OTSU)求二值化阈值
uint8 otsu_threshold(uint8 image[][MT9V03X_W])
{
    uint32 histogram[256] = {0};
    uint32 total = MT9V03X_H * MT9V03X_W;

    for (uint16 i = 0; i < MT9V03X_H; i++)
        for (uint16 j = 0; j < MT9V03X_W; j++)
            histogram[image[i][j]]++;

    uint32 total_sum = 0;
    for (uint16 i = 0; i < 256; i++)
        total_sum += i * histogram[i];

    float max_variance = 0;
    uint8 best_threshold = 0;
    uint32 w0 = 0;
    uint32 sum0 = 0;

    for (uint16 t = 0; t < 256; t++)
    {
        w0 += histogram[t];
        if (w0 == 0) continue;
        if (w0 == total) break;

        sum0 += t * histogram[t];
        uint32 w1 = total - w0;
        uint32 sum1 = total_sum - sum0;

        float mu0 = (float)sum0 / w0;
        float mu1 = (float)sum1 / w1;
        float variance = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);

        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = (uint8)t;
        }
    }

    return best_threshold;
}

//根据图像阈值进行二值化处理
void set_image_twovalues(uint8 thr)
{
    for (uint16 i = 0; i < MT9V03X_H; i++)
    {
        for (uint16 j = 0; j < MT9V03X_W; j++)
        {
            if (base_image[i][j] < thr)
            {
                twovalues_image[i][j] = 0; // 将像素值设置为黑色
            }
            else 
            {
                twovalues_image[i][j] = 255; // 将像素值设置为白色
            }
        }
    }
}





//找出图像基点
void find_base_point(void)
{
    uint8 row = MT9V03X_H -1;

    // 优先用图像正中间(W/2)
    if ( twovalues_image[row][MT9V03X_W / 2] == 255
        && twovalues_image[row][MT9V03X_W / 2 + 1] == 255
        && twovalues_image[row][MT9V03X_W / 2 - 1] == 255)
    {
        for (uint16 i = MT9V03X_W / 2; i > 0; i--)
        {
            if (twovalues_image[row][i - 1] == 0 && twovalues_image[row][i] == 255 && twovalues_image[row][i + 1] == 255)
            {
                base_point_left = i + 1;
                break;
            }
        }
        for (uint16 i = MT9V03X_W / 2; i < MT9V03X_W; i++)
        {
            if (twovalues_image[row][i] == 0 && twovalues_image[row][i - 1] == 255 && twovalues_image[row][i - 2] == 255)
            {
                base_point_right = i - 1;
                break;
            }
        }
    }

    // 中间被遮挡时，用左侧四分之一处
    else if (twovalues_image[row][MT9V03X_W / 4] == 255
        && twovalues_image[row][MT9V03X_W / 4 + 1] == 255
        && twovalues_image[row][MT9V03X_W / 4 - 1] == 255)
    {
        for (uint16 i = MT9V03X_W / 4; i > 0; i--)
        {
            if (twovalues_image[row][i - 1] == 0 && twovalues_image[row][i] == 255 && twovalues_image[row][i + 1] == 255)
            {
                base_point_left = i + 1;
                break;
            }
        }
        for (uint16 i = MT9V03X_W / 4; i < MT9V03X_W; i++)
        {
            if (twovalues_image[row][i] == 0 && twovalues_image[row][i - 1] == 255 && twovalues_image[row][i - 2] == 255)
            {
                base_point_right = i - 1;
                break;
            }
        }
    }

    // 最后尝试右侧四分之三处
    else if (twovalues_image[row][MT9V03X_W / 4 * 3] == 255
        && twovalues_image[row][MT9V03X_W / 4 * 3 + 1] == 255
        && twovalues_image[row][MT9V03X_W / 4 * 3 - 1] == 255)
    {
        for (uint16 i = MT9V03X_W / 4 * 3; i > 0; i--)
        {
            if (twovalues_image[row][i - 1] == 0 && twovalues_image[row][i] == 255 && twovalues_image[row][i + 1] == 255)
            {
                base_point_left = i + 1;
                break;
            }
        }
        for (uint16 i = MT9V03X_W / 4 * 3; i < MT9V03X_W; i++)
        {
            if (twovalues_image[row][i] == 0 && twovalues_image[row][i - 1] == 255 && twovalues_image[row][i - 2] == 255)
            {
                base_point_right = i - 1;
                break;
            }
        }
    }
}

//根据二值化数组找到赛道边界
void find_boundary(void)
{
     uint8 left_point=base_point_left;    // 左边界从基点开始搜
     uint8 right_point=base_point_right;  // 右边界从基点开始搜
     for(uint16 i=MT9V03X_H-2;i>search_end_line;i--)  // 从下往上搜
     {
        uint8 flag_leftpoint_left_search=0;  // 标记左边界点向右搜索范围的最右边还没找到左边界点(开始向左搜索)
        uint8 flag_leftpoint_mid_search=0;  // 标记左边界点向左搜索范围的最左边还没找到左边界点（开始由中间向左搜索）
        uint8 flag_rightpoint_right_search=0;  // 标记右边界点向左搜索范围的最左边还没找到右边界点(开始向右搜索)
        uint8 flag_rightpoint_mid_search=0;  // 标记右边界点向右搜索范围的最右边还没找到右边界点（开始由中间向右搜索）
        
         for(uint8 j=left_point;j<left_point+left_search_right_range;j++)  // 搜索左边界
         {
            //向右10个像素搜索左边界
             if(twovalues_image[i][j]==0&&twovalues_image[i][j+1]==255&&twovalues_image[i][j+2]==255)  // 找到左边界点
             {
                 left_point=j;
                 break;
             }
             if(j+2==MT9V03X_W-1)  // 如果搜索到图像最右边还没找到左边界点，则将左边界点设置为图像最右边-2
             {
                 left_point=MT9V03X_W-3;
                 break;
             }
             if(j==left_point+left_search_right_range-1)  // 如果搜索到左边界搜索范围的最右边还没找到左边界点，则将左边界点设置为搜索范围的最右边
             {
                flag_leftpoint_left_search=1;  // 标记左边界点搜索范围的最右边还没找到左边界点
             }
         }
         //向左5个像素点搜索左边界
            if(flag_leftpoint_left_search==1)  // 如果左边界点向右搜索范围的最右边还没找到左边界点，则向左5个像素点搜索左边界
            {
                for(uint8 j=left_point;j>left_point-left_search_left_range;j--)  // 搜索左边界
                {
                    if(twovalues_image[i][j]==255&&twovalues_image[i][j-1]==0&&twovalues_image[i][j-2]==0)  // 找到左边界点
                    {
                        left_point=j;
                        break;
                    }
                    if(j==2)  // 如果搜索到图像最左边还没找到左边界点，则将左边界点设置为图像最左边+2
                    {
                        left_point=2;
                        break;
                    }
                    if(j==left_point-left_search_left_range+1)  // 如果搜索到左边界向右搜索范围的最左边还没找到左边界点，则向右5个像素点搜索左边界
                    {
                        flag_leftpoint_mid_search=1;  // 标记左边界点搜索范围的最左边还没找到左边界点
                    }
                }
            }
        //由中间向左搜索左边界
            if(flag_leftpoint_mid_search==1)  // 如果左边界点向左搜索范围的最左边还没找到左边界点，则由中间向左搜索左边界
            {
                for(uint8 j=94;j>0;j--)  // 搜索左边界
                {
                    if(twovalues_image[i][j]==255&&twovalues_image[i][j-1]==0&&twovalues_image[i][j-2]==0)  // 找到左边界点
                    {
                        left_point=j;
                        break;
                    }
                    if(j==2)  // 如果搜索到图像最左边还没找到左边界点，则将左边界点设置为图像最左边+2
                    {
                        left_point=2;
                        break;
                    }
                }
            }
        //向左10个像素点搜索右边界
         for(uint8 j=right_point;j>right_point-right_search_left_range;j--)  // 搜索右边界
         {
             if(twovalues_image[i][j]==0&&twovalues_image[i][j-1]==255&&twovalues_image[i][j-2]==255)  // 找到右边界点
             {
                 right_point=j;
                 break;
             }
             if(j==2)  // 如果搜索到图像最左边还没找到右边界点，则将右边界点设置为图像最左边+2
             {
                 right_point=2;
                 break;
             }
             if(j==right_point-right_search_left_range+1)  // 如果搜索到右边界向左搜索范围的最左边还没找到右边界点，则向右5个像素点搜索右边界
             {
                flag_rightpoint_right_search=1;  // 标记右边界点搜索范围的最左边还没找到右边界点
             }
         }
         //向右5个像素点搜索右边界
            if(flag_rightpoint_right_search==1)  // 如果右边界点向左搜索范围的最左边还没找到右边界点，则向右5个像素点搜索右边界
            {
                for(uint8 j=right_point;j<right_point+right_search_right_range;j++)  // 搜索右边界
                {
                    if(twovalues_image[i][j]==255&&twovalues_image[i][j+1]==0&&twovalues_image[i][j+2]==0)  // 找到右边界点
                    {
                        right_point=j;
                        break;
                    }
                    if(j+2==MT9V03X_W-1)  // 如果搜索到图像最右边还没找到右边界点，则将右边界点设置为图像最右边-2
                    {
                        right_point=MT9V03X_W-3;
                        break;
                    }
                    if(j==right_point+right_search_right_range-1)  // 如果搜索到右边界向右搜索范围的最右边还没找到右边界点，则由中间向右搜索右边界
                    {
                        flag_rightpoint_mid_search=1;  // 标记右边界点搜索范围的最右边还没找到右边界点
                    }
                }
            }
        //由中间向右搜索右边界
            if(flag_rightpoint_mid_search==1)  // 如果右边界点向右搜索范围的最右边还没找到右边界点，则由中间向右搜索右边界
            {
                for(uint8 j=94;j<MT9V03X_W;j++)  // 搜索右边界
                {
                    if(twovalues_image[i][j]==255&&twovalues_image[i][j+1]==0&&twovalues_image[i][j+2]==0)  // 找到右边界点
                    {
                        right_point=j;
                        break;
                    }
                    if(j==MT9V03X_W-3)  // 如果搜索到图像最右边还没找到右边界点，则将右边界点设置为图像最右边-2
                    {
                        right_point=MT9V03X_W-3;
                        break;
                    }
                }
            }
        left_line[i]=uint8_limit(left_point, 0, MT9V03X_W-1);  // 将左边界点存入数组
        right_line[i]=uint8_limit(right_point, 0, MT9V03X_W-1);  // 将右边界点存入数组
        mid_line[i]=uint8_limit((left_point+right_point)/2, 0, MT9V03X_W-1);  // 将中间点存入数组
    }
}



//ips200屏上画边线
void draw_boundary(void)
{
    for (uint16 i = search_end_line; i < MT9V03X_H; i++)
    {
        ips200_draw_point(left_line[i], 100 + i, RGB565_RED);
        ips200_draw_point(right_line[i], 100 + i, RGB565_BLUE);
        ips200_draw_point(mid_line[i], 100 + i, RGB565_GREEN);
    }
}

//环岛检测状态机
void ring_state_process(void)
{
    static uint8 ring_state = ring_state_idle;  // 环岛状态机状态变量
    static uint16 ring_counter = 0;             // 环岛计数器

    switch (ring_state)
    {
        case ring_state_idle:  // 初始状态，未检测到环岛
            if (enter_detect_ring())
            {
                ring_state = ring_state_detecting;
                ring_counter = 0;
            }
            break;

        case ring_state_detecting:  // 连续确认入口特征
            if (enter_detect_ring())
            {
                ring_counter++;
                if (ring_counter > RING_DETECTION_THRESHOLD1)
                {
                    // 入口特征确认后，先不改中线，等待右近处完全丢线。
                    ring_state = ring_state_wait_right_near_lost;
                    ring_counter = 0;
                }
            }
            else
            {
                ring_state = ring_state_idle;
                ring_counter = 0;
            }
            break;

        case ring_state_wait_right_near_lost:  // 等右近处完全丢线后，才开始信任左边线行驶
            if (right_near_full_lost_detect())
            {
                ring_counter++;
                if (ring_counter > RING_RIGHT_NEAR_LOST_CONFIRM_THRESHOLD)
                {
                    ring_state = ring_state_left_straight;
                    ring_counter = 0;
                }
            }
            else
            {
                ring_counter = 0;
            }
            break;

        case ring_state_left_straight:  // 信任左边界行驶，等待右近处不丢线且中远处丢线 60% 后再拉入环线
            left_boundary_straight_handle();
            if (right_entry_ready_detect())
            {
                ring_counter++;
                if (ring_counter > RING_ENTRY_CONFIRM_THRESHOLD)
                {
                    ring_state = ring_state_entry;
                    ring_counter = 0;
                }
            }
            else
            {
                ring_counter = 0;
            }
            break;

        case ring_state_entry:  // 根据中远处外圆边界画入环线
            enter_ring_handle();
            ring_counter++;
            if (ring_counter > RING_DETECTION_THRESHOLD2)
            {
                ring_state = ring_state_in;
                ring_counter = 0;
            }
            break;

        case ring_state_in:  // 环内
            in_ring_handle();
            if (exit_detect_ring())
            {
                ring_state = ring_state_confirmed;
                ring_counter = 0;
            }
            break;

        case ring_state_confirmed:  // 连续确认是否已经出环
            exit_ring_handle();
            if (exit_detect_ring())
            {
                ring_counter++;
                if (ring_counter > RING_DETECTION_THRESHOLD3)
                {
                    ring_state = ring_state_idle;
                    ring_counter = 0;
                }
            }
            else
            {
                ring_state = ring_state_in;
                ring_counter = 0;
            }
            break;

        default:
            ring_state = ring_state_idle;
            ring_counter = 0;
            break;
    }
}


static bool enter_detect_ring(void)
{
    // 右环岛入口检测：右边界远处丢线、近处不丢线，同时左边线 90% 以上不丢线。
    // 行坐标越大越靠近车，越小越远。
    uint8 right_far_lost;
    uint8 right_near_lost;
    uint8 left_lost;

    const uint8 far_start = 35;
    const uint8 far_end = 70;
    const uint8 near_start = 85;
    const uint8 near_end = MT9V03X_H - 1;
    const uint8 left_start = 30;
    const uint8 left_end = MT9V03X_H - 1;

    const uint8 far_rows = far_end - far_start + 1;
    const uint8 near_rows = near_end - near_start + 1;
    const uint8 left_rows = left_end - left_start + 1;

    lost_location_detection(1, far_start, far_end);
    right_far_lost = lost_counter;

    lost_location_detection(1, near_start, near_end);
    right_near_lost = lost_counter;

    lost_location_detection(0, left_start, left_end);
    left_lost = lost_counter;

    // 远处右边界至少 70% 丢线；近处右边界最多 20% 丢线；左边界至少 90% 有效。
    if ((uint16)right_far_lost * 100 >= (uint16)far_rows * 55 &&
        (uint16)right_near_lost * 100 <= (uint16)near_rows * 20 &&
        (uint16)left_lost * 100 <= (uint16)left_rows * 10)
    {
        return true;
    }

    return false;
}


static bool right_near_full_lost_detect(void)
{
    // 只有右边界近处基本完全丢线后，才允许进入“信左边界行驶”状态。
    // 这里用 90% 近处行丢线作为“完全丢线”的抗噪判据。
    uint8 right_near_lost;

    const uint8 near_start = 90;
    const uint8 near_end = MT9V03X_H - 1;
    const uint8 near_rows = near_end - near_start + 1;

    lost_location_detection(1, near_start, near_end);
    right_near_lost = lost_counter;

    if ((uint16)right_near_lost * 100 >= (uint16)near_rows * 90)
    {
        return true;
    }

    return false;
}


static bool right_entry_ready_detect(void)
{
    // 由“信左边界行驶”切到 entry 拉线阶段的条件：
    // 1. 右边界近处不丢线；
    // 2. 右边界中远处丢线达到 60%。
    uint8 right_mid_far_lost;
    uint8 right_near_lost;

    const uint8 mid_far_start = 35;
    const uint8 mid_far_end = 85;
    const uint8 near_start = 90;
    const uint8 near_end = MT9V03X_H - 1;

    const uint8 mid_far_rows = mid_far_end - mid_far_start + 1;
    const uint8 near_rows = near_end - near_start + 1;

    lost_location_detection(1, mid_far_start, mid_far_end);
    right_mid_far_lost = lost_counter;

    lost_location_detection(1, near_start, near_end);
    right_near_lost = lost_counter;

    // 中远处右边界丢线 >= 60%；近处右边界丢线 <= 20%，即近处基本不丢线。
    if ((uint16)right_mid_far_lost * 100 >= (uint16)mid_far_rows * 50 &&
        (uint16)right_near_lost * 100 <= (uint16)near_rows * 20)
    {
        return true;
    }

    return false;
}


static bool exit_detect_ring(void)
{
    // 出环特征：右边界重新回到图像右侧，左右边界宽度恢复到普通赛道宽度。
    uint8 right_valid_counter = 0;
    uint8 width_valid_counter = 0;

    for (uint8 i = 55; i < MT9V03X_H - 5; i++)
    {
        if (right_line[i] > MT9V03X_W - 35 && right_line[i] < MT9V03X_W - 2)
        {
            right_valid_counter++;
        }

        if (right_line[i] > left_line[i] &&
            (right_line[i] - left_line[i]) > 60 &&
            (right_line[i] - left_line[i]) < 125)
        {
            width_valid_counter++;
        }
    }

    return (right_valid_counter > 30 && width_valid_counter > 25);
}


static void left_boundary_straight_handle(void)
{
    // 右环岛入口确认后，先信任左边界直行：不使用此时可能错误的 right_line。
    // 偏移不再是固定值，而是按行查表：远处小、近处大。
    ring_offset_table_init();
    left_boundary_array_offset_handle(ring_left_straight_offset_table);
}


static void left_boundary_array_offset_handle(const uint8 offset_table[])
{
    for (uint8 i = search_end_line; i < MT9V03X_H; i++)
    {
        uint8 offset = offset_table[i];
        if (left_line[i] > 2 && left_line[i] < MT9V03X_W - offset - 2)
        {
            int16 mid = (int16)left_line[i] + offset;
            mid_line[i] = uint8_limit(mid, 0, MT9V03X_W - 1);
        }
    }
}


static void ring_offset_table_init(void)
{
    if (ring_offset_table_inited)
    {
        return;
    }

    for (uint8 i = 0; i < MT9V03X_H; i++)
    {
        ring_left_straight_offset_table[i] = ring_calc_row_offset(i, RING_LEFT_STRAIGHT_OFFSET_FAR, RING_LEFT_STRAIGHT_OFFSET_NEAR);
        ring_left_enter_offset_table[i] = ring_calc_row_offset(i, RING_LEFT_ENTER_OFFSET_FAR, RING_LEFT_ENTER_OFFSET_NEAR);
    }

    ring_offset_table_inited = 1;
}


static uint8 ring_calc_row_offset(uint8 row, uint8 far_offset, uint8 near_offset)
{
    if (row <= search_end_line)
    {
        return far_offset;
    }
    if (row >= MT9V03X_H - 1)
    {
        return near_offset;
    }

    return (uint8)(far_offset +
        ((uint16)(near_offset - far_offset) * (uint16)(row - search_end_line)) /
        (uint16)(MT9V03X_H - 1 - search_end_line));
}


static void enter_ring_handle(void)
{
    // entry 阶段：当前 left_line 仍可能是直道线，right_line 中远处也可能丢线/误线。
    // 所以直接从二值图中远处搜索环岛外圆边界，画一条入环引导边线，再根据这条边线生成中线。
    ring_outer_entry_line_handle();
}


static void ring_outer_entry_line_handle(void)
{
    uint8 outer_row;
    uint8 outer_col;

    ring_offset_table_init();

    if (!find_right_ring_outer_point(&outer_row, &outer_col))
    {
        // 外圆点还没稳定找到时，退回左边界直行，避免使用错误的右线。
        left_boundary_straight_handle();
        return;
    }

    uint8 start_row = MT9V03X_H - 1;
    uint8 start_col = left_line[start_row];

    if (start_col <= 2 || start_col >= MT9V03X_W - 3)
    {
        start_col = left_line[110];
    }
    if (start_col <= 2 || start_col >= MT9V03X_W - 3)
    {
        start_col = base_point_left;
    }

    // outer_col 是中远处外圆边界点，start_col 是近处可用左边界点。
    // 将两点连成一条“入环边线”，写回 left_line，并由它补出 mid_line。
    for (uint8 i = search_end_line; i < MT9V03X_H; i++)
    {
        int16 entry_left;

        if (i <= outer_row)
        {
            entry_left = outer_col;
        }
        else
        {
            int16 numerator = (int16)(start_col - outer_col) * (int16)(i - outer_row);
            int16 denominator = (int16)(start_row - outer_row);
            if (denominator <= 0)
            {
                denominator = 1;
            }
            entry_left = (int16)outer_col + numerator / denominator;
        }

        left_line[i] = uint8_limit(entry_left, 0, MT9V03X_W - 1);
        mid_line[i] = uint8_limit(entry_left + ring_left_enter_offset_table[i], 0, MT9V03X_W - 1);
    }
}


static bool find_right_ring_outer_point(uint8 *outer_row, uint8 *outer_col)
{
    uint16 row_sum = 0;
    uint16 col_sum = 0;
    uint8 point_count = 0;

    for (uint8 i = RING_OUTER_SEARCH_TOP; i <= RING_OUTER_SEARCH_BOTTOM; i++)
    {
        uint8 min_col = 2;
        uint8 max_col = MT9V03X_W - 3;

        // 避开当前普通左线，优先找比直道左线更靠右的黑->白跳变，这更像右环岛外圆边界。
        if (left_line[i] + RING_OUTER_MIN_LEFT_GAP > min_col &&
            left_line[i] + RING_OUTER_MIN_LEFT_GAP < max_col)
        {
            min_col = left_line[i] + RING_OUTER_MIN_LEFT_GAP;
        }

        for (uint8 j = min_col; j <= max_col; j++)
        {
            // 外圆左边界通常仍是 黑->白：0,255,255。
            if (twovalues_image[i][j - 1] == 0 &&
                twovalues_image[i][j] == 255 &&
                twovalues_image[i][j + 1] == 255)
            {
                row_sum += i;
                col_sum += j;
                point_count++;
                break;
            }
        }
    }

    if (point_count >= RING_OUTER_MIN_POINTS)
    {
        *outer_row = (uint8)(row_sum / point_count);
        *outer_col = (uint8)(col_sum / point_count);
        return true;
    }

    return false;
}


static void in_ring_handle(void)
{
    // 环岛内：继续以内圆边界为参考绕行，偏移稍大，避免贴内圆。
    ring_right_inner_line_handle(RING_RIGHT_IN_OFFSET);
}


static void exit_ring_handle(void)
{
    // 出环阶段：减小偏移，帮助中线逐渐回到普通巡线结果。
    ring_right_inner_line_handle(RING_RIGHT_EXIT_OFFSET);
}


static void ring_right_inner_line_handle(int16 offset)
{
    for (uint8 i = search_end_line; i < MT9V03X_H; i++)
    {
        if (right_line[i] > offset + 2 && right_line[i] < MT9V03X_W - 3)
        {
            int16 mid = (int16)right_line[i] - offset;
            mid_line[i] = uint8_limit(mid, 0, MT9V03X_W - 1);
        }
    }
}


//丢线检测函数
static void lost_location_detection(uint8 l_or_r, uint8 start_row, uint8 end_row)
{
    lost_counter = 0;

    if (start_row >= MT9V03X_H)
    {
        return;
    }
    if (end_row >= MT9V03X_H)
    {
        end_row = MT9V03X_H - 1;
    }

    if (l_or_r == 0)  // 检测左边界
    {
        for (uint8 i = start_row; i <= end_row; i++)
        {
            // 左丢线兜底可能是 0 或 2。
            if (left_line[i] <= 2)
            {
                lost_counter++;
            }
        }
    }
    else if (l_or_r == 1)  // 检测右边界
    {
        for (uint8 i = start_row; i <= end_row; i++)
        {
            // 右丢线兜底可能是 MT9V03X_W-1 或 MT9V03X_W-3。
            if (right_line[i] >= MT9V03X_W - 3)
            {
                lost_counter++;
            }
        }
    }
}
