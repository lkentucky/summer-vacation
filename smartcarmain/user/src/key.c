#include "key.h"

#include "mymenu.h"
#include "zf_device_key.h"

void key_handle(void) {
  key_scanner();

  key_state_enum s1 = key_get_state(KEY_1);
  key_state_enum s2 = key_get_state(KEY_2);
  key_state_enum s3 = key_get_state(KEY_3);
  key_state_enum s4 = key_get_state(KEY_4);

  if (s1 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_1);
    key_1();
  }
  if (s2 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_2);
    key_2();
  }
  if (s3 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_3);
    enter_folder();
    enter_editting();
  }
  if (s4 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_4);
    back_folder();
  }
}
