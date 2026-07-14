#ifndef __MENU_H_
#define __MENU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct MenuItem MenuItem;  // 提前声明，不然结构体内指针自引会报错

typedef enum MenuKind MenuKind;

enum MenuKind {
  menu_folder = 0,
  int32_box,
  int16_box,
  int8_box,
  uint16_box,
  uint8_box,
  float_box,
  bool_box,
};

struct MenuItem {
  const char* name;

  void* data;     // 指向变量
  MenuKind kind;  // 记录数据种类
  bool editing;   // 记录当前是否处于编辑状态

  uint8_t number_of_sons;  // 记录当前子节点个数
  uint8_t seq;             // 记录当前是父节点的第几个成员

  MenuItem* father;
  MenuItem* first_son;
  MenuItem* next_brother;
  MenuItem* prev_brother;
};

void create_menu_item(MenuItem* father, MenuItem* me, const char name[],
                      void* data, MenuKind kind);
void create_menu_folder(MenuItem* father, MenuItem* me, const char name[]);
void create_menu_txt(MenuItem* father, MenuItem* me, const char name[],
                     void* data, MenuKind kind);

#endif
