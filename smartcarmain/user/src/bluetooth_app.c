#include "bluetooth_app.h"

#include "IMU.h"
#include "cross.h"
#include "isr.h"
#include "motor.h"
#include "zf_device_type.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLUETOOTH_APP_RX_BUFFER_SIZE       (256)
#define BLUETOOTH_APP_COMMAND_SIZE         (128)
#define BLUETOOTH_APP_TX_SIZE              (220)
#define BLUETOOTH_APP_AT_TIMEOUT_MS        (180)
#define BLUETOOTH_APP_STARTUP_DELAY_MS     (350)
#define BLUETOOTH_APP_TELEMETRY_MS_MIN     (100)
#define BLUETOOTH_APP_TELEMETRY_MS_MAX     (2000)

uint8  bluetooth_app_ready = 0;
uint8  bluetooth_app_connected = 0;
uint32 bluetooth_app_baud = 0;
int    bluetooth_app_telemetry_ms = BLUETOOTH_APP_TELEMETRY_MS_DEFAULT;

extern int image_fps;
extern int image_frame_ms;
extern int zebra_cross_count;

static volatile uint8 bluetooth_app_rx_buffer[BLUETOOTH_APP_RX_BUFFER_SIZE];
static volatile uint16 bluetooth_app_rx_head = 0;
static volatile uint16 bluetooth_app_rx_tail = 0;
static char bluetooth_app_command[BLUETOOTH_APP_COMMAND_SIZE];
static uint16 bluetooth_app_command_length = 0;
static volatile uint32 bluetooth_app_last_rx_tick = 0;
static uint8 bluetooth_app_stream_enabled = 0;
static uint32 bluetooth_app_last_telemetry_tick = 0;

typedef enum
{
    BLUETOOTH_PARAM_INT,
    BLUETOOTH_PARAM_FLOAT
} bluetooth_param_type_enum;

typedef struct
{
    const char *name;
    bluetooth_param_type_enum type;
    void *address;
    float minimum;
    float maximum;
} bluetooth_param_struct;

static bluetooth_param_struct bluetooth_app_parameters[] =
{
    {"SPEED_KP",       BLUETOOTH_PARAM_FLOAT, &Kp,                               0.0f, 100.0f},
    {"SPEED_KI",       BLUETOOTH_PARAM_FLOAT, &Ki,                               0.0f, 20.0f},
    {"SPEED_KD",       BLUETOOTH_PARAM_FLOAT, &Kd,                               0.0f, 20.0f},
    {"RUN_SPEED",      BLUETOOTH_PARAM_INT,   &run_base_speed,                   0.0f, 600.0f},
    {"VISION_KP",      BLUETOOTH_PARAM_FLOAT, &vision_yaw_kp,                    0.0f, 20.0f},
    {"VISION_KD",      BLUETOOTH_PARAM_FLOAT, &vision_yaw_kd,                    0.0f, 2.0f},
    {"VISION_FF",      BLUETOOTH_PARAM_FLOAT, &vision_yaw_kff,                 -10.0f, 10.0f},
    {"YAW_KP",         BLUETOOTH_PARAM_FLOAT, &yaw_rate_kp,                      0.0f, 10.0f},
    {"YAW_SIGN",       BLUETOOTH_PARAM_FLOAT, &yaw_rate_feedback_sign,          -2.0f, 2.0f},
    {"YAW_MAX",        BLUETOOTH_PARAM_INT,   &yaw_rate_limit_dps,               0.0f, 720.0f},
#if SPEED_DECISION_ENABLE
    {"STRAIGHT_SPEED", BLUETOOTH_PARAM_INT,   &speed_straight_speed,             0.0f, 600.0f},
    {"CORNER_SPEED",   BLUETOOTH_PARAM_INT,   &speed_corner_speed,               0.0f, 600.0f},
    {"STRAIGHT_VKP",   BLUETOOTH_PARAM_FLOAT, &speed_straight_vision_kp,         0.0f, 20.0f},
    {"CORNER_VKP",     BLUETOOTH_PARAM_FLOAT, &speed_corner_vision_kp,           0.0f, 20.0f},
    {"STRAIGHT_YKP",   BLUETOOTH_PARAM_FLOAT, &speed_straight_yaw_rate_kp,       0.0f, 10.0f},
    {"CORNER_YKP",     BLUETOOTH_PARAM_FLOAT, &speed_corner_yaw_rate_kp,         0.0f, 10.0f},
    {"ENTER_PX",       BLUETOOTH_PARAM_FLOAT, &SPEED_ENTER_LINE_PX,              0.0f, 94.0f},
    {"EXIT_PX",        BLUETOOTH_PARAM_FLOAT, &SPEED_EXIT_LINE_PX,               0.0f, 94.0f},
    {"ACCEL_STEP",     BLUETOOTH_PARAM_FLOAT, &speed_accel_step,                 0.0f, 100.0f},
    {"DECEL_STEP",     BLUETOOTH_PARAM_FLOAT, &speed_decel_step,                 0.0f, 100.0f},
#endif
};

static uint16 bluetooth_app_rx_next(uint16 index)
{
    index++;
    if (index >= BLUETOOTH_APP_RX_BUFFER_SIZE) index = 0;
    return index;
}

void bluetooth_app_uart_callback(void)
{
    uint8 data;
    uint16 next;

    if (!uart_query_byte(BLUETOOTH_APP_UART_INDEX, &data)) return;
    next = bluetooth_app_rx_next(bluetooth_app_rx_head);
    if (next == bluetooth_app_rx_tail) return; // 缓冲区满时丢弃新字节，避免覆盖尚未处理的命令。
    bluetooth_app_rx_buffer[bluetooth_app_rx_head] = data;
    bluetooth_app_rx_head = next;
    bluetooth_app_last_rx_tick = g_sys_tick;
}

static uint8 bluetooth_app_rx_pop(uint8 *data)
{
    if (bluetooth_app_rx_tail == bluetooth_app_rx_head) return 0;
    *data = bluetooth_app_rx_buffer[bluetooth_app_rx_tail];
    bluetooth_app_rx_tail = bluetooth_app_rx_next(bluetooth_app_rx_tail);
    return 1;
}

static void bluetooth_app_rx_clear(void)
{
    bluetooth_app_rx_tail = bluetooth_app_rx_head;
}

static void bluetooth_app_uart_start(uint32 baud)
{
    bluetooth_app_rx_clear();
    uart_init(BLUETOOTH_APP_UART_INDEX, baud,
              BLUETOOTH_APP_UART_TX_PIN, BLUETOOTH_APP_UART_RX_PIN);
    uart_rx_interrupt(BLUETOOTH_APP_UART_INDEX, 1);
    bluetooth_app_baud = baud;
}

static uint8 bluetooth_app_wait_text(const char *expected, uint32 timeout_ms)
{
    char response[48];
    uint16 length = 0;
    uint8 data;

    response[0] = '\0';
    while (timeout_ms-- > 0)
    {
        while (bluetooth_app_rx_pop(&data))
        {
            if (length < sizeof(response) - 1)
                response[length++] = (char)data;
            else
            {
                memmove(response, response + 1, sizeof(response) - 2);
                response[sizeof(response) - 2] = (char)data;
                length = sizeof(response) - 1;
            }
            response[length] = '\0';
            if (strstr(response, expected) != NULL) return 1;
        }
        system_delay_ms(1);
    }
    return 0;
}

static uint8 bluetooth_app_test_at(void)
{
    bluetooth_app_rx_clear();
    uart_write_string(BLUETOOTH_APP_UART_INDEX, "AT"); // HC-04 AT指令不能带回车换行。
    return bluetooth_app_wait_text("OK", BLUETOOTH_APP_AT_TIMEOUT_MS);
}

static uint8 bluetooth_app_configure_uart(void)
{
    // 优先尝试目标波特率，避免模块已配置后每次上电重复改写参数。
    bluetooth_app_uart_start(BLUETOOTH_APP_TARGET_BAUD);
    if (bluetooth_app_test_at()) return 0;

    // 出厂默认9600N81。确认后发送永久保存的波特率设置指令，模块会自动重启。
    bluetooth_app_uart_start(BLUETOOTH_APP_FACTORY_BAUD);
    if (!bluetooth_app_test_at()) return 1;

    bluetooth_app_rx_clear();
    uart_write_string(BLUETOOTH_APP_UART_INDEX, "AT+BAUD=115200");
    if (!bluetooth_app_wait_text("OK", BLUETOOTH_APP_AT_TIMEOUT_MS))
    {
        // 设置失败仍可按出厂9600通信，只把遥测频率降到1Hz。
        bluetooth_app_telemetry_ms = 1000;
        return 0;
    }

    system_delay_ms(BLUETOOTH_APP_STARTUP_DELAY_MS);
    bluetooth_app_uart_start(BLUETOOTH_APP_TARGET_BAUD);
    if (!bluetooth_app_test_at()) return 1;
    return 0;
}

static int32 bluetooth_app_round_tenths(float value)
{
    return (int32)(value * 10.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static void bluetooth_app_uppercase(char *text)
{
    while (*text != '\0')
    {
        *text = (char)toupper((unsigned char)*text);
        text++;
    }
}

static char *bluetooth_app_skip_spaces(char *text)
{
    while (*text == ' ' || *text == '\t') text++;
    return text;
}

static void bluetooth_app_trim_end(char *text)
{
    size_t length = strlen(text);
    while (length > 0 && (text[length - 1] == ' ' || text[length - 1] == '\t' ||
                           text[length - 1] == '\r' || text[length - 1] == '\n'))
        text[--length] = '\0';
}

static void bluetooth_app_send(const char *text)
{
    // 命令应答不能依赖STATE脚，否则转接板未引出STATE或接触不良时会表现为发送没反应。
    if (!bluetooth_app_ready || text == NULL) return;
    uart_write_buffer(BLUETOOTH_APP_UART_INDEX, (const uint8 *)text, (uint32)strlen(text));
}

static bluetooth_param_struct *bluetooth_app_find_parameter(const char *name)
{
    uint32 i;
    for (i = 0; i < sizeof(bluetooth_app_parameters) / sizeof(bluetooth_app_parameters[0]); i++)
        if (strcmp(name, bluetooth_app_parameters[i].name) == 0) return &bluetooth_app_parameters[i];
    return NULL;
}

static void bluetooth_app_send_parameter(const bluetooth_param_struct *parameter)
{
    char response[80];

    if (parameter->type == BLUETOOTH_PARAM_INT)
        snprintf(response, sizeof(response), "P,%s=%d\n", parameter->name,
                 *((int *)parameter->address));
    else
    {
        float value = *((float *)parameter->address);
        int32 value_x1000 = (int32)(value * 1000.0f + (value >= 0.0f ? 0.5f : -0.5f));
        int32 absolute_value = (value_x1000 < 0) ? -value_x1000 : value_x1000;
        snprintf(response, sizeof(response), "P,%s=%s%ld.%03ld\n", parameter->name,
                 (value_x1000 < 0) ? "-" : "", (long)(absolute_value / 1000),
                 (long)(absolute_value % 1000));
    }
    bluetooth_app_send(response);
}

static void bluetooth_app_send_status(void)
{
    char response[BLUETOOTH_APP_TX_SIZE];
    snprintf(response, sizeof(response),
             "S,run=%d,base=%d,vl10=%ld,vr10=%ld,err=%d,gyro10=%ld,fps=%d,state=%d,stream=%d,rate=%d,baud=%lu\n",
             (base_speed > 0 || joystick_control_active), base_speed,
             (long)bluetooth_app_round_tenths(real_speedl),
             (long)bluetooth_app_round_tenths(real_speedr),
             (int)steering_get_image_error(),
             (long)bluetooth_app_round_tenths(imu_gyro_z_dps_filter), image_fps,
#if SPEED_DECISION_ENABLE
             speed_state,
#else
             0,
#endif
             bluetooth_app_stream_enabled, bluetooth_app_telemetry_ms,
             (unsigned long)bluetooth_app_baud);
    bluetooth_app_send(response);
}

static void bluetooth_app_stop_car(void)
{
    motor_joystick_stop();
    cross_state_reset();
}

static void bluetooth_app_start_car(void)
{
    bluetooth_app_stop_car();
#if SPEED_DECISION_ENABLE
    base_speed = speed_decision_speed;
#else
    base_speed = run_base_speed;
#endif
}

static void bluetooth_app_set_parameter(char *arguments)
{
    char *name = bluetooth_app_skip_spaces(arguments);
    char *value_text = name;
    char *end;
    float value;
    bluetooth_param_struct *parameter;

    while (*value_text != '\0' && *value_text != ' ' && *value_text != '\t') value_text++;
    if (*value_text == '\0')
    {
        bluetooth_app_send("ERR,usage:SET NAME VALUE\n");
        return;
    }
    *value_text = '\0';
    value_text = bluetooth_app_skip_spaces(value_text + 1);
    bluetooth_app_uppercase(name);

    parameter = bluetooth_app_find_parameter(name);
    if (parameter == NULL)
    {
        bluetooth_app_send("ERR,unknown parameter\n");
        return;
    }

    value = strtof(value_text, &end);
    end = bluetooth_app_skip_spaces(end);
    if (end == value_text || *end != '\0')
    {
        bluetooth_app_send("ERR,invalid value\n");
        return;
    }
    if (value < parameter->minimum || value > parameter->maximum)
    {
        bluetooth_app_send("ERR,out of range\n");
        return;
    }

    if (parameter->type == BLUETOOTH_PARAM_INT)
        *((int *)parameter->address) = (int)value;
    else
        *((float *)parameter->address) = value;
    bluetooth_app_send_parameter(parameter);
}

static void bluetooth_app_get_parameter(char *arguments)
{
    char *name = bluetooth_app_skip_spaces(arguments);
    bluetooth_param_struct *parameter;

    bluetooth_app_trim_end(name);
    bluetooth_app_uppercase(name);
    if (*name == '\0')
    {
        bluetooth_app_send_status();
        return;
    }
    parameter = bluetooth_app_find_parameter(name);
    if (parameter == NULL)
        bluetooth_app_send("ERR,unknown parameter\n");
    else
        bluetooth_app_send_parameter(parameter);
}

static uint8 bluetooth_app_process_joystick(char *command)
{
    int turn;
    int forward;
    int unused_1;
    int unused_2;
    int consumed = 0;
    char *end;

    if (command[0] != '[') return 0;
    bluetooth_app_uppercase(command);
    if (strncmp(command, "[JOYSTICK", 9) != 0) return 0;

    if (sscanf(command, "[JOYSTICK , %d , %d , %d , %d ] %n",
               &turn, &forward, &unused_1, &unused_2, &consumed) != 4)
    {
        bluetooth_app_send("ERR,joystick format [joystick,x,y,x,x]\n");
        return 1;
    }
    end = bluetooth_app_skip_spaces(command + consumed);
    if (*end != '\0')
    {
        bluetooth_app_send("ERR,joystick format [joystick,x,y,x,x]\n");
        return 1;
    }
    if (turn < -100 || turn > 100 || forward < -100 || forward > 100 ||
        unused_1 < -100 || unused_1 > 100 || unused_2 < -100 || unused_2 > 100)
    {
        bluetooth_app_send("ERR,joystick range -100..100\n");
        return 1;
    }

    motor_joystick_set(turn, forward);
    return 1;
}
static void bluetooth_app_process_command(char *command)
{
    char *arguments;
    char *space;

    bluetooth_app_trim_end(command);
    command = bluetooth_app_skip_spaces(command);
    if (*command == '\0' || strncmp(command, "OK+", 3) == 0) return;
    // 无STATE模式下，断开后HC-04会把遥测当AT命令并返回ERR；收到后立即停止回传。
    if (strncmp(command, "ERR", 3) == 0)
    {
        bluetooth_app_stream_enabled = 0;
        motor_joystick_stop();
        return;
    }
    if (bluetooth_app_process_joystick(command)) return;

    space = command;
    while (*space != '\0' && *space != ' ' && *space != '\t') space++;
    arguments = space;
    if (*space != '\0')
    {
        *space = '\0';
        arguments = bluetooth_app_skip_spaces(space + 1);
    }
    bluetooth_app_uppercase(command);

    if (strcmp(command, "PING") == 0)
        bluetooth_app_send("OK,PONG\n");
    else if (strcmp(command, "STATUS") == 0)
        bluetooth_app_send_status();
    else if (strcmp(command, "GET") == 0)
        bluetooth_app_get_parameter(arguments);
    else if (strcmp(command, "SET") == 0)
        bluetooth_app_set_parameter(arguments);
    else if (strcmp(command, "START") == 0)
    {
        bluetooth_app_start_car();
        bluetooth_app_send("OK,START\n");
    }
    else if (strcmp(command, "STOP") == 0)
    {
        bluetooth_app_stop_car();
        bluetooth_app_send("OK,STOP\n");
    }
    else if (strcmp(command, "STREAM") == 0)
    {
        bluetooth_app_uppercase(arguments);
        if (strcmp(arguments, "ON") == 0)
        {
            bluetooth_app_stream_enabled = 1;
            bluetooth_app_send("OK,STREAM=ON\n");
        }
        else if (strcmp(arguments, "OFF") == 0)
        {
            bluetooth_app_stream_enabled = 0;
            bluetooth_app_send("OK,STREAM=OFF\n");
        }
        else
            bluetooth_app_send("ERR,usage:STREAM ON|OFF\n");
    }
    else if (strcmp(command, "RATE") == 0)
    {
        char *end;
        long minimum = (bluetooth_app_baud <= BLUETOOTH_APP_FACTORY_BAUD) ? 500 : BLUETOOTH_APP_TELEMETRY_MS_MIN;
        long period = strtol(arguments, &end, 10);
        end = bluetooth_app_skip_spaces(end);
        if (end == arguments || *end != '\0' || period < minimum || period > BLUETOOTH_APP_TELEMETRY_MS_MAX)
            bluetooth_app_send("ERR,RATE range 100..2000ms (9600baud min 500)\n");
        else
        {
            char response[40];
            bluetooth_app_telemetry_ms = (int)period;
            snprintf(response, sizeof(response), "OK,RATE=%d\n", bluetooth_app_telemetry_ms);
            bluetooth_app_send(response);
        }
    }
    else if (strcmp(command, "HELP") == 0)
        bluetooth_app_send("CMD:PING STATUS GET [NAME] SET NAME VALUE START STOP STREAM ON|OFF RATE 100..2000 [joystick,x,y,x,x]\n");
    else
        bluetooth_app_send("ERR,unknown command; send HELP\n");
}

static void bluetooth_app_receive(void)
{
    uint8 data;
    while (bluetooth_app_rx_pop(&data))
    {
        // 兼容江协小程序的CR、LF、CRLF三种发送结束符。
        if (data == '\r' || data == '\n')
        {
            if (bluetooth_app_command_length > 0)
            {
                bluetooth_app_command[bluetooth_app_command_length] = '\0';
                bluetooth_app_process_command(bluetooth_app_command);
            }
            bluetooth_app_command_length = 0;
        }
        else if (data == ']' && bluetooth_app_command_length > 0 &&
                 bluetooth_app_command[0] == '[')
        {
            if (bluetooth_app_command_length < BLUETOOTH_APP_COMMAND_SIZE - 1)
            {
                bluetooth_app_command[bluetooth_app_command_length++] = (char)data;
                bluetooth_app_command[bluetooth_app_command_length] = '\0';
                bluetooth_app_process_command(bluetooth_app_command);
            }
            bluetooth_app_command_length = 0;
        }
        else if (bluetooth_app_command_length < BLUETOOTH_APP_COMMAND_SIZE - 1)
            bluetooth_app_command[bluetooth_app_command_length++] = (char)data;
        else
            bluetooth_app_command_length = 0;
    }

    // 小程序关闭发送新行时，利用30ms字节间隔作为一包结束。
    if (bluetooth_app_command_length > 0 &&
        (g_sys_tick - bluetooth_app_last_rx_tick) >= (30U + SYS_TICK_MS - 1U) / SYS_TICK_MS)
    {
        bluetooth_app_command[bluetooth_app_command_length] = '\0';
        bluetooth_app_process_command(bluetooth_app_command);
        bluetooth_app_command_length = 0;
    }
}

static void bluetooth_app_send_telemetry(void)
{
    char telemetry[BLUETOOTH_APP_TX_SIZE];
    snprintf(telemetry, sizeof(telemetry),
             "B,t=%lu,run=%d,b=%d,tl10=%ld,tr10=%ld,vl10=%ld,vr10=%ld,e=%d,g10=%ld,yr10=%ld,fps=%d,fms=%d,st=%d,cr=%u,zb=%d\n",
             (unsigned long)(g_sys_tick * SYS_TICK_MS), (base_speed > 0 || joystick_control_active), base_speed,
             (long)bluetooth_app_round_tenths(target_speedl),
             (long)bluetooth_app_round_tenths(target_speedr),
             (long)bluetooth_app_round_tenths(real_speedl),
             (long)bluetooth_app_round_tenths(real_speedr),
             (int)steering_get_image_error(),
             (long)bluetooth_app_round_tenths(imu_gyro_z_dps_filter),
             (long)bluetooth_app_round_tenths(yaw_rate_ref_dps), image_fps, image_frame_ms,
#if SPEED_DECISION_ENABLE
             speed_state,
#else
             0,
#endif
             (unsigned int)cross_state, zebra_cross_count);
    bluetooth_app_send(telemetry);
}

uint8 bluetooth_app_init(void)
{
#if BLUETOOTH_APP_ENABLE
    bluetooth_app_ready = 0;
    bluetooth_app_connected = 0;
    bluetooth_app_stream_enabled = 0;
    bluetooth_app_telemetry_ms = BLUETOOTH_APP_TELEMETRY_MS_DEFAULT;
    bluetooth_app_rx_head = 0;
    bluetooth_app_rx_tail = 0;
    bluetooth_app_command_length = 0;
    bluetooth_app_last_rx_tick = g_sys_tick;

#if BLUETOOTH_APP_USE_STATE_PIN
    gpio_init(BLUETOOTH_APP_STATE_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
#endif
    set_wireless_type(WIRELESS_UART, bluetooth_app_uart_callback);

    if (bluetooth_app_configure_uart())
    {
        // 配置失败时回到出厂波特率，保留接收能力，但向主程序报告初始化失败。
        bluetooth_app_uart_start(BLUETOOTH_APP_FACTORY_BAUD);
        bluetooth_app_telemetry_ms = 1000;
        return 1;
    }

    bluetooth_app_ready = 1;
    bluetooth_app_last_telemetry_tick = g_sys_tick;
    return 0;
#else
    return 1;
#endif
}

void bluetooth_app_process(void)
{
#if BLUETOOTH_APP_ENABLE
    uint8 connected_now;
    uint32 period_ticks;

    if (!bluetooth_app_ready) return;
#if BLUETOOTH_APP_USE_STATE_PIN
    connected_now = (gpio_get_level(BLUETOOTH_APP_STATE_PIN) == BLUETOOTH_APP_STATE_CONNECTED_LEVEL);
#else
    connected_now = 1;
#endif
    if (connected_now && !bluetooth_app_connected)
        bluetooth_app_last_telemetry_tick = g_sys_tick;
    bluetooth_app_connected = connected_now;

    bluetooth_app_receive();
    if (!bluetooth_app_connected || !bluetooth_app_stream_enabled) return;

    period_ticks = (uint32)((bluetooth_app_telemetry_ms + SYS_TICK_MS - 1) / SYS_TICK_MS);
    if ((g_sys_tick - bluetooth_app_last_telemetry_tick) >= period_ticks)
    {
        bluetooth_app_last_telemetry_tick = g_sys_tick;
        bluetooth_app_send_telemetry();
    }
#endif
}
