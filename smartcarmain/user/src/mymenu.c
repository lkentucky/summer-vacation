#include "mymenu.h"

#include "menu.h"
#include "zf_device_ips200.h"
#include "zf_driver_pit.h"
#include "motor.h"
#include "IMU.h"

MenuItem head;
MenuItem* current_index;


bool state = true;
uint8 threshold = 230;
int pwm = 1000;
extern int image_period_ms;
extern int image_frame_ms;
extern int image_proc_ms;
extern int image_fps;
extern int image_wait_count;

void Init_menu(void) {
  // 初始化head
  head.name = "head";
  head.father = NULL;
  head.first_son = NULL;
  head.prev_brother = NULL;
  head.next_brother = NULL;
  head.number_of_sons = 0;
  head.data = NULL;
  head.kind = menu_folder;

  MenuItem* pid_folder = dynamic_create_menu_folder(&head, "PID");
  MenuItem* motor_folder = dynamic_create_menu_folder(&head, "motor");
  MenuItem* xunxian_folder = dynamic_create_menu_folder(&head, "xunxian");
  MenuItem* image_folder = dynamic_create_menu_folder(&head, "image");



  dynamic_create_menu_txt(pid_folder, "Kp", &Kp, float_box);
  dynamic_create_menu_txt(pid_folder, "Ki", &Ki, float_box);
  dynamic_create_menu_txt(pid_folder, "Kd", &Kd, float_box);
  dynamic_create_menu_txt(pid_folder, "state", &state, bool_box);
  //dynamic_create_menu_txt(motor_folder, "PWM", &pwm, int32_box);
  dynamic_create_menu_txt(motor_folder, "target_speedl", &target_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "target_speedr", &target_speedr, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedl", &real_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedr", &real_speedr, float_box);
  dynamic_create_menu_txt(xunxian_folder, "run_speed", &run_base_speed, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "Kp_steer", &Kp_steer, float_box);
  dynamic_create_menu_txt(xunxian_folder, "Kd_steer_position", &Kd_steer_position, float_box);
  dynamic_create_menu_txt(xunxian_folder, "Kd_steer_time", &Kd_steer_time, float_box);
  dynamic_create_menu_txt(xunxian_folder, "Kgyro_steer", &Kgyro_steer, float_box);
  dynamic_create_menu_txt(xunxian_folder, "gyro_z", &imu_gyro_z_dps_filter, float_box);
  dynamic_create_menu_txt(image_folder, "period_ms", &image_period_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "frame_ms", &image_frame_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "proc_ms", &image_proc_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "fps", &image_fps, int32_box);
  dynamic_create_menu_txt(image_folder, "wait", &image_wait_count, int32_box);
  dynamic_create_menu_txt(&head, "threshold", &threshold, uint8_box);

  current_index = head.first_son;
}

void Show_array(void) {
  MenuItem* h = current_index->father;
  MenuItem* s = h->first_son;

  for (int i = 0; i < h->number_of_sons; i++) {
    if (s == current_index)
      ips200_show_string(0, 16 * i, "->");
    else
      ips200_show_string(0, 16 * i, "  ");
    s = s->next_brother;
  }
}

void Show_txt(void) {
  MenuItem* f = current_index->father;
  MenuItem* s = f->first_son;

  if (current_index->kind == menu_folder) return;
  if (current_index->editing) {
    ips200_show_string(142, current_index->seq * 16, "|");
  } else {
    ips200_show_string(142, current_index->seq * 16, " ");
  }

  for (int i = 0; i < f->number_of_sons; i++) {
    switch (s->kind) {
      case int32_box:
        ips200_show_int(150, i * 16, *(int32*)s->data, 5);
        break;
      case float_box:
        ips200_show_float(150, i * 16, *(float*)s->data, 5, 2);
        break;
      case bool_box:
        if (*(bool*)s->data) {
          ips200_show_string(150, i * 16, "on ");
        } else {
          ips200_show_string(150, i * 16, "off");
        }
      case uint8_box:
        ips200_show_int(150, i * 16, *(uint8*)s->data, 3);
        break;
      default:
        break;
    }

    s = s->next_brother;
  }
}

void Show_menu(void) {
  MenuItem* f = current_index->father;
  MenuItem* s = f->first_son;

  for (int i = 0; i < f->number_of_sons; i++) {
    ips200_show_string(16, 16 * i, s->name);
    s = s->next_brother;
  }
  Show_txt();
  Show_array();
}

void array_up(void) {
  if (current_index->prev_brother != NULL) {
    current_index = current_index->prev_brother;
  }
}

void array_down(void) {
  if (current_index->next_brother != NULL) {
    current_index = current_index->next_brother;
  }
}

void enter_folder(void) {
  if (current_index->kind == menu_folder && current_index->first_son != NULL) {
    ips200_clear();
    current_index = current_index->first_son;
  }
}

void back_folder(void) {
  if (current_index->father != NULL) {
    ips200_clear();
    current_index = current_index->father;
  }
}

// 进入/退出编辑状态(K3)
void enter_editting(void) {
  if (current_index->kind != menu_folder) {
    current_index->editing = !current_index->editing;
  }
}

// key_1()和key_2()函数用于处理按键K1和K2的操作。当当前菜单项处于编辑状态时，按下K1会增加数值，按下K2会减少数值；当不处于编辑状态时，按下K1会向上移动选择，按下K2会向下移动选择。
void key_1(void) {
  if (current_index->editing) {
    switch (current_index->kind) {
      case int32_box:
        (*(int32*)current_index->data)++;
        break;
      case float_box:
        (*(float*)current_index->data) += 0.01;
        break;
      case bool_box:
        (*(bool*)current_index->data) = !(*(bool*)current_index->data);
      case uint8_box:
        (*(uint8*)current_index->data)++;
      default:
        break;
}
  } else {
    array_up();
  }
}

void key_2(void) {
  if (current_index->editing) {
    switch (current_index->kind) {
      case int32_box:
        (*(int32*)current_index->data)--;
        break;
      case float_box:
        (*(float*)current_index->data) -= 0.01;
        break;
      case bool_box:
        (*(bool*)current_index->data) = !(*(bool*)current_index->data);
      case uint8_box:
        (*(uint8*)current_index->data)--;
      default:
        break;
    }
  } else {
    array_down();
  }
}

void key_3(void) {
  if (current_index->kind == menu_folder && current_index->first_son != NULL) {
    enter_folder();
  } else {
    enter_editting();
  }
}

void key_3_double(void) {
  if (current_index->father != NULL) {
    ips200_clear();
    current_index = &head;
  }
}

void key_4_double(void) {
  motor_pid_reset();
  target_speedl = 0.0f;
  target_speedr = 0.0f;

  if (base_speed == 0) {
    base_speed = run_base_speed;  // 使用菜单中的 run_speed，不再写死固定速度
  } else {
    base_speed = 0;
  }
}

