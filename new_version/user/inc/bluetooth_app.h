#ifndef __BLUETOOTH_APP_H_
#define __BLUETOOTH_APP_H_

#include "zf_common_typedef.h"

#define BLUETOOTH_APP_ENABLE                 (1)

// HC-04 UART6：MCU C6(TX)->模块RXD，模块TXD->MCU C7(RX)。
#define BLUETOOTH_APP_UART_INDEX             (UART_6)
#define BLUETOOTH_APP_UART_TX_PIN            (UART6_TX_C6)
#define BLUETOOTH_APP_UART_RX_PIN            (UART6_RX_C7)
#define BLUETOOTH_APP_TARGET_BAUD            (115200)
#define BLUETOOTH_APP_FACTORY_BAUD           (9600)

// 当前硬件未接STATE，软件由STREAM ON/OFF控制回传；今后接入STATE可改为1。
#define BLUETOOTH_APP_USE_STATE_PIN          (0)
#define BLUETOOTH_APP_STATE_PIN              (C13)
#define BLUETOOTH_APP_STATE_CONNECTED_LEVEL  (GPIO_LOW)

#define BLUETOOTH_APP_TELEMETRY_MS_DEFAULT   (200)

extern uint8  bluetooth_app_ready;
extern uint8  bluetooth_app_connected;
extern uint32 bluetooth_app_baud;
extern int    bluetooth_app_telemetry_ms;

// 自动探测115200；若模块仍是出厂9600，则用HC-04 AT指令改为115200。
uint8 bluetooth_app_init(void);

// 主循环中高频调用，处理透明串口命令和周期遥测。
void bluetooth_app_process(void);

// 由UART6中断经wireless_module_uart_handler调用。
void bluetooth_app_uart_callback(void);

#endif
