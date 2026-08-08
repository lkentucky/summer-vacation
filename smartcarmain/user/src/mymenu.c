#include "mymenu.h"

#include "menu.h"
#include "zf_device_ips200.h"
#include "zf_driver_pit.h"
#include "motor.h"
#include "IMU.h"
#include "cross.h"

MenuItem head;
MenuItem* current_index;
static MenuItem* motor_folder = NULL;
static MenuItem* image_folder = NULL;


bool state = true;
uint8 threshold = 230;
int pwm = 1000;
extern int image_period_ms;
extern int image_frame_ms;
extern int image_proc_ms;
extern int image_fps;
extern int image_wait_count;
extern int zebra_cross_count;       // 本次运行已经通过的斑马线数量
extern int zebra_transition_count;  // 底部三行中单行“白黑黑”次数的最大值
extern int zebra_match_row_count;   // 底部三行中“白黑黑”次数达到4的行数
extern int zebra_state;             // 斑马线状态：0等待、1确认、2通过

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
  motor_folder = dynamic_create_menu_folder(&head, "motor");
  MenuItem* xunxian_folder = dynamic_create_menu_folder(&head, "xunxian");
  image_folder = dynamic_create_menu_folder(&head, "image");



  dynamic_create_menu_txt(pid_folder, "Kp", &Kp, float_box);
  dynamic_create_menu_txt(pid_folder, "Ki", &Ki, float_box);
  dynamic_create_menu_txt(pid_folder, "Kd", &Kd, float_box);
  dynamic_create_menu_txt(pid_folder, "state", &state, bool_box);
  //dynamic_create_menu_txt(motor_folder, "PWM", &pwm, int32_box);
  dynamic_create_menu_txt(motor_folder, "target_speedl", &target_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "target_speedr", &target_speedr, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedl", &real_speedl, float_box);
  dynamic_create_menu_txt(motor_folder, "real_speedr", &real_speedr, float_box);
  dynamic_create_menu_txt(motor_folder, "enc_l_raw", (void *)&encoder_test_total_l, int32_box);
  dynamic_create_menu_txt(motor_folder, "enc_r_raw", (void *)&encoder_test_total_r, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "run_speed", &run_base_speed, int32_box);
  // 串级方向控制参数：视觉PD外环生成yaw_ref，角速度P内环跟踪yaw_ref。
  dynamic_create_menu_txt(xunxian_folder, "vision_kd", &vision_yaw_kd, float_box);
  dynamic_create_menu_txt(xunxian_folder, "vision_ff", &vision_yaw_kff, float_box);
  dynamic_create_menu_txt(xunxian_folder, "yaw_max", &yaw_rate_limit_dps, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "gyro_z", &imu_gyro_z_dps_filter, float_box);
#if SPEED_DECISION_ENABLE
  // 三状态速度决策菜单：spd_state中0=直道，1=弯道，2=摆动抑制。
 // dynamic_create_menu_txt(xunxian_folder, "spd_straight", &speed_straight_speed, int32_box);
 // dynamic_create_menu_txt(xunxian_folder, "spd_corner", &speed_corner_speed, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "yaw_str", &speed_straight_yaw_feedback_sign, float_box);
  dynamic_create_menu_txt(xunxian_folder, "yaw_cur", &speed_corner_yaw_feedback_sign, float_box);
  dynamic_create_menu_txt(xunxian_folder, "ykp_str", &speed_straight_yaw_rate_kp, float_box);
  dynamic_create_menu_txt(xunxian_folder, "ykp_cur", &speed_corner_yaw_rate_kp, float_box);
  dynamic_create_menu_txt(xunxian_folder, "kp_str", &speed_straight_vision_kp, float_box);
  dynamic_create_menu_txt(xunxian_folder, "kp_cur", &speed_corner_vision_kp, float_box);
  dynamic_create_menu_txt(xunxian_folder, "spd_state", &speed_state, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "spd_cmd", &speed_decision_speed, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "spd_up", &speed_accel_step, float_box);
  dynamic_create_menu_txt(xunxian_folder, "spd_down", &speed_decel_step, float_box);
  dynamic_create_menu_txt(xunxian_folder, "straight_n", &speed_straight_confirm_frames, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "corner_n", &speed_corner_confirm_frames, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "osc_n", &speed_oscillation_reversal_required, int32_box);
  dynamic_create_menu_txt(xunxian_folder, "enter_px", &SPEED_ENTER_LINE_PX, float_box);
  dynamic_create_menu_txt(xunxian_folder, "exit_px", &SPEED_EXIT_LINE_PX, float_box);
#else
  dynamic_create_menu_txt(xunxian_folder, "vision_kp", &vision_yaw_kp, float_box);
  dynamic_create_menu_txt(xunxian_folder, "yaw_kp", &yaw_rate_kp, float_box);
#endif
  dynamic_create_menu_txt(image_folder, "period_ms", &image_period_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "frame_ms", &image_frame_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "proc_ms", &image_proc_ms, int32_box);
  dynamic_create_menu_txt(image_folder, "fps", &image_fps, int32_box);
  dynamic_create_menu_txt(image_folder, "wait", &image_wait_count, int32_box);
  dynamic_create_menu_txt(image_folder, "cross", &cross_state, uint8_box);
  dynamic_create_menu_txt(image_folder, "zebra_n", &zebra_cross_count, int32_box);
  dynamic_create_menu_txt(image_folder, "zebra_wbb", &zebra_transition_count, int32_box);
  dynamic_create_menu_txt(image_folder, "zebra_rows", &zebra_match_row_count, int32_box);
  dynamic_create_menu_txt(image_folder, "zebra_state", &zebra_state, int32_box);
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

bool menu_is_image_page(void) {
  return image_folder != NULL &&
         current_index != NULL &&
         current_index->father == image_folder;
}

bool menu_is_motor_page(void) {
  return motor_folder != NULL &&
         current_index != NULL &&
         current_index->father == motor_folder;
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
  uint8 was_running = (joystick_control_active || base_speed != 0);

  motor_joystick_stop();
  if (!was_running) {
#if SPEED_DECISION_ENABLE
    // motor_joystick_stop已复位速度决策，避免启动瞬间出现速度尖峰。
    base_speed = speed_decision_speed;
#else
    base_speed = run_base_speed;
#endif
  }
}

