#include "key.h"

#include "mymenu.h"
#include "zf_device_key.h"
#include "zf_driver_gpio.h"

#define DOUBLE_CLICK_TICKS 30
#define INITIAL_DELAY 50
#define REPEAT_DELAY 10

static uint32_t tick = 0;
static uint32_t key3_press_tick = 0;
static uint32_t key4_press_tick = 0;
static bool key3_pending = false;
static bool key4_pending = false;
static uint8_t key1_repeat = 0;
static uint8_t key2_repeat = 0;
static bool key1_held = false;
static bool key2_held = false;

void key_handle(void) {
  key_scanner();
  tick++;

  if (key3_pending && (tick - key3_press_tick > DOUBLE_CLICK_TICKS)) {
    key_3();
    key3_pending = false;
  }
  if (key4_pending && (tick - key4_press_tick > DOUBLE_CLICK_TICKS)) {
    back_folder();
    key4_pending = false;
  }

  key_state_enum s1 = key_get_state(KEY_1);
  key_state_enum s2 = key_get_state(KEY_2);
  key_state_enum s3 = key_get_state(KEY_3);
  key_state_enum s4 = key_get_state(KEY_4);

  if (s1 == KEY_SHORT_PRESS || s1 == KEY_LONG_PRESS) {
    key_clear_state(KEY_1);
  }
  if (s2 == KEY_SHORT_PRESS || s2 == KEY_LONG_PRESS) {
    key_clear_state(KEY_2);
  }
  if (s3 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_3);
    if (key3_pending) {
      key_3_double();
      key3_pending = false;
    } else {
      key3_pending = true;
      key3_press_tick = tick;
    }
  }
  if (s4 == KEY_SHORT_PRESS) {
    key_clear_state(KEY_4);
    if (key4_pending) {
      key_4_double();
      key4_pending = false;
    } else {
      key4_pending = true;
      key4_press_tick = tick;
    }
  }

  if (gpio_get_level(E2) == GPIO_LOW) {
    if (!key1_held) {
      key1_held = true;
      key_1();
      key1_repeat = INITIAL_DELAY;
    } else if (key1_repeat > 0) {
      key1_repeat--;
      if (key1_repeat == 0) {
        key_1();
        key1_repeat = REPEAT_DELAY;
      }
    }
  } else {
    key1_held = false;
    key1_repeat = 0;
  }

  if (gpio_get_level(E3) == GPIO_LOW) {
    if (!key2_held) {
      key2_held = true;
      key_2();
      key2_repeat = INITIAL_DELAY;
    } else if (key2_repeat > 0) {
      key2_repeat--;
      if (key2_repeat == 0) {
        key_2();
        key2_repeat = REPEAT_DELAY;
      }
    }
  } else {
    key2_held = false;
    key2_repeat = 0;
  }
}
