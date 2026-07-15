#ifndef __MYMENU_H_
#define __MYMENU_H_

#include "menu.h"
#include "zf_device_ips200.h"

extern int threshold;  // 二值化阈值
void Init_menu(void);
void Show_menu(void);
void Show_array(void);
void Show_txt(void);
void key_1(void);
void key_2(void);
void key_3(void);
void key_3_double(void);
void enter_folder(void);
void back_folder(void);
void enter_editting(void);
#endif
