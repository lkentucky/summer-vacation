#include "mymenu.h"
#include "zf_driver_pit.h"
#include "motor.h"
#include "IMU.h"
#include "image.h"
#include "cross.h"

MenuItem head;
MenuItem *current_index;
static MenuItem *motor_folder = NULL;
static MenuItem *image_folder = NULL;

bool state = true;
uint8 threshold = 128U;
int pwm = 1000;

extern int image_period_ms;
extern int image_frame_ms;
extern int image_proc_ms;
extern int image_fps;
extern int image_wait_count;
extern int zebra_cross_count;
extern int zebra_transition_count;
extern int zebra_match_row_count;
extern int zebra_state;

void Init_menu(void)
{
    MenuItem *pid_folder;
    MenuItem *yaw_folder;
    MenuItem *vision_folder;

    head.name = "head";
    head.father = NULL;
    head.first_son = NULL;
    head.prev_brother = NULL;
    head.next_brother = NULL;
    head.number_of_sons = 0U;
    head.data = NULL;
    head.kind = menu_folder;
    head.editing = false;

    pid_folder = dynamic_create_menu_folder(&head, "speed_pid");
    motor_folder = dynamic_create_menu_folder(&head, "motor");
    yaw_folder = dynamic_create_menu_folder(&head, "steer_ppdd");
    vision_folder = dynamic_create_menu_folder(&head, "vision");
    image_folder = dynamic_create_menu_folder(&head, "image");

    dynamic_create_menu_txt(pid_folder, "Kp", &Kp, float_box);
    dynamic_create_menu_txt(pid_folder, "Ki", &Ki, float_box);
    dynamic_create_menu_txt(pid_folder, "Kd", &Kd, float_box);
    dynamic_create_menu_txt(pid_folder, "enabled", &state, bool_box);

    dynamic_create_menu_txt(motor_folder, "target_l", &target_speedl, float_box);
    dynamic_create_menu_txt(motor_folder, "target_r", &target_speedr, float_box);
    dynamic_create_menu_txt(motor_folder, "real_l", &real_speedl, float_box);
    dynamic_create_menu_txt(motor_folder, "real_r", &real_speedr, float_box);
    dynamic_create_menu_txt(motor_folder, "enc_l", (void *)&encoder_test_total_l, int32_box);
    dynamic_create_menu_txt(motor_folder, "enc_r", (void *)&encoder_test_total_r, int32_box);

    dynamic_create_menu_txt(yaw_folder, "gyro_z", &imu_gyro_z_dps_filter, float_box);
    dynamic_create_menu_txt(yaw_folder, "steer_err", &track_steering_value, float_box);
    dynamic_create_menu_txt(yaw_folder, "steer_out", (void *)&steer_ppdd_output, float_box);
    dynamic_create_menu_txt(yaw_folder, "steer_kp", &steer_ppdd_kp, float_box);
    dynamic_create_menu_txt(yaw_folder, "steer_kd", &steer_ppdd_kd, float_box);
    dynamic_create_menu_txt(yaw_folder, "gyro_k", &steer_ppdd_gyro_k, float_box);

    dynamic_create_menu_txt(vision_folder, "width_k", &mid_single_edge_width_scale, float_box);
    dynamic_create_menu_txt(vision_folder, "recent_w", &track_recent_width_px, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_0", &speed_straight_speed, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_1", &speed_mid_fast_speed, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_2", &speed_mid_slow_speed, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_3", &speed_corner_speed, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_lv", &speed_level, int32_box);
    dynamic_create_menu_txt(vision_folder, "speed_cmd", &speed_decision_speed, int32_box);
    dynamic_create_menu_txt(vision_folder, "launch_n", &speed_launch_frames, int32_box);
    dynamic_create_menu_txt(vision_folder, "launch_i", &speed_launch_count, int32_box);

    dynamic_create_menu_txt(image_folder, "threshold", &threshold, uint8_box);
    dynamic_create_menu_txt(image_folder, "period_ms", &image_period_ms, int32_box);
    dynamic_create_menu_txt(image_folder, "frame_ms", &image_frame_ms, int32_box);
    dynamic_create_menu_txt(image_folder, "proc_ms", &image_proc_ms, int32_box);
    dynamic_create_menu_txt(image_folder, "fps", &image_fps, int32_box);
    dynamic_create_menu_txt(image_folder, "confidence", &track_confidence, uint8_box);
    dynamic_create_menu_txt(image_folder, "steer", &track_steering_value, float_box);
    dynamic_create_menu_txt(image_folder, "cross", &cross_state, uint8_box);
    dynamic_create_menu_txt(image_folder, "zebra_n", &zebra_cross_count, int32_box);
    dynamic_create_menu_txt(image_folder, "zebra_wbb", &zebra_transition_count, int32_box);
    dynamic_create_menu_txt(image_folder, "zebra_rows", &zebra_match_row_count, int32_box);
    dynamic_create_menu_txt(image_folder, "zebra_state", &zebra_state, int32_box);
    dynamic_create_menu_txt(image_folder, "wait", &image_wait_count, int32_box);

    current_index = head.first_son;
}

void Show_array(void)
{
    MenuItem *parent = current_index->father;
    MenuItem *item;
    int index;

    if (parent == NULL) return;
    item = parent->first_son;
    for (index = 0; index < parent->number_of_sons; index++)
    {
        ips200_show_string(0, 16 * index, (item == current_index) ? "->" : "  ");
        item = item->next_brother;
    }
}

void Show_txt(void)
{
    MenuItem *parent = current_index->father;
    MenuItem *item;
    int index;

    if (parent == NULL || current_index->kind == menu_folder) return;
    ips200_show_string(142, current_index->seq * 16,
                       current_index->editing ? "|" : " ");
    item = parent->first_son;
    for (index = 0; index < parent->number_of_sons; index++)
    {
        switch (item->kind)
        {
            case int32_box:
                ips200_show_int(150, index * 16, *(int32 *)item->data, 5);
                break;
            case int16_box:
                ips200_show_int(150, index * 16, *(int16 *)item->data, 5);
                break;
            case int8_box:
                ips200_show_int(150, index * 16, *(int8 *)item->data, 4);
                break;
            case uint16_box:
                ips200_show_int(150, index * 16, *(uint16 *)item->data, 5);
                break;
            case uint8_box:
                ips200_show_int(150, index * 16, *(uint8 *)item->data, 3);
                break;
            case float_box:
                ips200_show_float(150, index * 16, *(float *)item->data, 5, 2);
                break;
            case bool_box:
                ips200_show_string(150, index * 16, *(bool *)item->data ? "on " : "off");
                break;
            default:
                break;
        }
        item = item->next_brother;
    }
}

void Show_menu(void)
{
    MenuItem *parent = current_index->father;
    MenuItem *item;
    int index;

    if (parent == NULL) parent = &head;
    item = parent->first_son;
    for (index = 0; index < parent->number_of_sons; index++)
    {
        ips200_show_string(16, 16 * index, item->name);
        item = item->next_brother;
    }
    Show_txt();
    Show_array();
}

bool menu_is_image_page(void)
{
    return image_folder != NULL && current_index != NULL &&
           current_index->father == image_folder;
}

bool menu_is_motor_page(void)
{
    return motor_folder != NULL && current_index != NULL &&
           current_index->father == motor_folder;
}

static void array_up(void)
{
    if (current_index->prev_brother != NULL) current_index = current_index->prev_brother;
}

static void array_down(void)
{
    if (current_index->next_brother != NULL) current_index = current_index->next_brother;
}

void enter_folder(void)
{
    if (current_index->kind == menu_folder && current_index->first_son != NULL)
    {
        ips200_clear();
        current_index = current_index->first_son;
    }
}

void back_folder(void)
{
    if (current_index->father != NULL)
    {
        ips200_clear();
        current_index = current_index->father;
    }
}

void enter_editting(void)
{
    if (current_index->kind != menu_folder) current_index->editing = !current_index->editing;
}

void key_1(void)
{
    if (!current_index->editing)
    {
        array_up();
        return;
    }
    switch (current_index->kind)
    {
        case int32_box: (*(int32 *)current_index->data)++; break;
        case int16_box: (*(int16 *)current_index->data)++; break;
        case int8_box: (*(int8 *)current_index->data)++; break;
        case uint16_box: (*(uint16 *)current_index->data)++; break;
        case uint8_box: (*(uint8 *)current_index->data)++; break;
        case float_box: (*(float *)current_index->data) += 0.01f; break;
        case bool_box: (*(bool *)current_index->data) = !(*(bool *)current_index->data); break;
        default: break;
    }
}

void key_2(void)
{
    if (!current_index->editing)
    {
        array_down();
        return;
    }
    switch (current_index->kind)
    {
        case int32_box: (*(int32 *)current_index->data)--; break;
        case int16_box: (*(int16 *)current_index->data)--; break;
        case int8_box: (*(int8 *)current_index->data)--; break;
        case uint16_box: (*(uint16 *)current_index->data)--; break;
        case uint8_box: (*(uint8 *)current_index->data)--; break;
        case float_box: (*(float *)current_index->data) -= 0.01f; break;
        case bool_box: (*(bool *)current_index->data) = !(*(bool *)current_index->data); break;
        default: break;
    }
}

void key_3(void)
{
    if (current_index->kind == menu_folder) enter_folder();
    else enter_editting();
}

void key_3_double(void)
{
    if (current_index->father != NULL)
    {
        ips200_clear();
        current_index = head.first_son;
    }
}

void key_4_double(void)
{
    uint8 was_running = (uint8)(joystick_control_active || base_speed != 0);
    motor_joystick_stop();
    if (!was_running)
    {
#if SPEED_DECISION_ENABLE
        base_speed = speed_decision_speed;
#else
        base_speed = run_base_speed;
#endif
    }
}
