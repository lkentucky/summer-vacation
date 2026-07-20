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
    uint8 row = MT9V03X_H -1; // 选择图像的第 110 行作为基点搜索行
    uint8 found = 0;

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
            if(i-1==0)
            {
                base_point_left=2;
                
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
            if(i+1==MT9V03X_W-1)
            {
                base_point_right=MT9V03X_W-3;
                
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
            if(i-1==0)
            {
                base_point_left=2;
                
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
            if(i+1==MT9V03X_W-1)
            {
                base_point_right=MT9V03X_W-3;
                
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
            if(i-1==0)
            {
                base_point_left=2;
                
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
            if(i+1==MT9V03X_W-1)
            {
                base_point_right=MT9V03X_W-3;
                
                break;
            }
        }
        
    }

    else
    {
        base_point_left = 0;
        base_point_right = MT9V03X_W - 1;
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
