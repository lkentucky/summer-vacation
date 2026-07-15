/*********************************************************************************************************************
 * MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK
 * 接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 MM32F327X-G8P 开源库的一部分
 *
 * MM32F327X-G8P 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即
 * GNU通用公共许可证）的条款 即 GPL 的第3版（即
 * GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt
 * 文件中 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.37
 * 适用平台          MM32F327X_G8P
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-08-10        Teternal            first version
 ********************************************************************************************************************/

#include "key.h"
#include "mymenu.h"
#include "zf_common_headfile.h"

// **************************** 代码区域 ****************************
#define IPS200_TYPE (IPS200_TYPE_SPI)

int main(void) {
  clock_init(SYSTEM_CLOCK_120M);  // 必须先初始化时钟
  debug_init();                   // 初始化 Debug UART

  ips200_init(IPS200_TYPE);  // 先初始化屏幕
  ips200_show_string(0, 304, "camera init...");

  while (mt9v03x_init()) {  // 初始化摄像头，失败则重试
    ips200_show_string(0, 304, "camera retry...");
    system_delay_ms(500);
  }
  ips200_show_string(0, 304, "camera ok     ");
  mt9v03x_set_exposure_time(512);         // 设置摄像头曝光时间
  mt9v03x_set_reg(MT9V03X_LR_OFFSET, 0);  // 设置摄像头左右偏移量
  mt9v03x_set_reg(MT9V03X_UD_OFFSET, 0);  // 设置摄像头上下偏移量
  mt9v03x_set_reg(MT9V03X_GAIN, 32);      // 设置摄像头图像增益
  mt9v03x_set_reg(MT9V03X_PCLK_MODE, 0);  // 设置摄像头像素时钟模式

  Init_menu();   // 初始化菜单数据
  key_init(10);  // 初始化按键扫描，10ms周期

  while (1) {
    key_handle();  // 处理按键输入
    Show_menu();   // 持续刷新屏幕
    if (mt9v03x_finish_flag) {
      mt9v03x_finish_flag = 0;
      ips200_show_gray_image(0, 100, mt9v03x_image[0], MT9V03X_W, MT9V03X_H,
                             188, 120, threshold);  // 显示摄像头图像
    }
    system_delay_ms(100);
  }
}
// **************************** 代码区域 ****************************
