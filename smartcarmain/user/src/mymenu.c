#include "mymenu.h"

#include "menu.h"
#include "zf_device_ips200.h"
#include "zf_driver_pit.h"

MenuItem head;
MenuItem* current_index;
// 文件夹
MenuItem m1;
MenuItem m2;
MenuItem m3;
MenuItem m4;
MenuItem m5;
// 文件（参数）
MenuItem m6;
MenuItem m7;
MenuItem m8;
MenuItem m9;

float Kp = 16;
float Ki = 3.14;
float Kd = 1;
bool state = true;
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

  create_menu_folder(&head, &m1, "menu1");
  create_menu_folder(&head, &m2, "menu2");
  create_menu_folder(&head, &m3, "menu3");
  create_menu_txt(&m1, &m6, "kp", &Kp, float_box);
  create_menu_txt(&m1, &m7, "ki", &Ki, float_box);
  create_menu_txt(&m1, &m8, "kd", &Kd, float_box);
  create_menu_txt(&m1, &m9, "state", &state, bool_box);

  create_menu_folder(&m1, &m4, "menu4");
  create_menu_folder(&m2, &m5, "menu5");
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
    ips200_show_string(72, current_index->seq * 16, "|");
  } else {
    ips200_show_string(72, current_index->seq * 16, " ");
  }

  for (int i = 0; i < f->number_of_sons; i++) {
    switch (s->kind) {
      case int32_box:
        ips200_show_int(80, i * 16, *(int32*)s->data, 5);
        break;
      case float_box:
        ips200_show_float(80, i * 16, *(float*)s->data, 5, 2);
        break;
      case bool_box:
        if (*(bool*)s->data) {
          ips200_show_string(80, i * 16, "on ");
        } else {
          ips200_show_string(80, i * 16, "off");
        }
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
      default:
        break;
    }
  } else {
    array_down();
  }
}