#include "menu.h"

// 初始化成员
void create_menu_item(MenuItem* father, MenuItem* me, const char name[],
                      void* data, MenuKind kind) {
  if (father->kind != menu_folder) return;

  me->name = name;
  me->father = father;
  me->first_son = NULL;
  me->prev_brother = NULL;
  me->next_brother = NULL;
  me->number_of_sons = 0;
  me->data = data;
  me->kind = kind;
  me->editing = false;

  if (father->number_of_sons == 0) {
    father->first_son = me;
  } else {
    MenuItem* p = father->first_son;
    while (p->next_brother != NULL) {
      p = p->next_brother;
    }
    p->next_brother = me;
    me->prev_brother = p;
  }
  // 序号等于当前父节点的子节点个数
  me->seq = father->number_of_sons;
  father->number_of_sons++;
}

// 创建文件夹
void create_menu_folder(MenuItem* father, MenuItem* me, const char name[]) {
  create_menu_item(father, me, name, NULL, menu_folder);
}

// 创建文件
void create_menu_txt(MenuItem* father, MenuItem* me, const char name[],
                     void* data, MenuKind kind) {
  create_menu_item(father, me, name, data, kind);
}
