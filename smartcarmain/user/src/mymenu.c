#include "mymenu.h"

#include "menu.h"
#include "zf_device_ips200.h"
#include "zf_driver_pit.h"
#include "motor.h"

MenuItem head;
MenuItem* current_index;


bool state = true;
uint8 threshold = 230;
int pwm = 1000;

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
  MenuItem* folder2 = dynamic_create_menu_folder(&head, "folder2");
  


  dynamic_create_menu_txt(pid_folder, "Kp", &Kp, float_box);
  dynamic_create_menu_txt(pid_folder, "Ki", &Ki, float_box);
  dynamic_create_menu_txt(pid_folder, "Kd", &Kd, float_box);
  dynamic_create_menu_txt(pid_folder, "state", &state, bool_box);
  //dynamic_create_menu_txt(motor_folder, "PWM", &pwm, int32_box);
  dynamic_create_menu_txt(motor_folder, "target_speedl", &target_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "target_speedr", &target_speedr, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedl", &real_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedr", &real_speedr, float_box);
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
    ips200_show_string(112, current_index->seq * 16, "|");
  } else {
    ips200_show_string(112, current_index->seq * 16, " ");
  }

  for (int i = 0; i < f->number_of_sons; i++) {
    switch (s->kind) {
      case int32_box:
        ips200_show_int(120, i * 16, *(int32*)s->data, 5);
        break;
      case float_box:
        ips200_show_float(120, i * 16, *(float*)s->data, 5, 2);
        break;
      case bool_box:
        if (*(bool*)s->data) {
          ips200_show_string(120, i * 16, "on ");
        } else {
          ips200_show_string(120, i * 16, "off");
        }
      case uint8_box:
        ips200_show_int(120, i * 16, *(uint8*)s->data, 3);  
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
  if (target_speedl == 0.0f) {
    target_speedl = 200.0f;
    //target_speedr = 200.0f;
  } else {
    target_speedl = 0.0f;
    //target_speedr = 0.0f;
  }
}

