#ifndef TEST_COMMON_H
#define TEST_COMMON_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int16_t int16;
typedef int32_t int32;
#define PI 3.14159265358979323846
#define float_abs fabsf
#define A0 0
#define A1 1
#define GPO 0
#define GPIO_HIGH 1
#define GPIO_LOW 0
#define GPO_PUSH_PULL 0
#define TIM5_PWM_CH3_A2 0
#define TIM5_PWM_CH4_A3 1
#define TIM3_ENCODER 3
#define TIM4_ENCODER 4
#define TIM3_ENCODER_CH1_B4 0
#define TIM3_ENCODER_CH2_B5 1
#define TIM4_ENCODER_CH1_B6 2
#define TIM4_ENCODER_CH2_B7 3
#define DEBUG_UART_INDEX 1
uint32 __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32 mask);
void gpio_init(int pin, int mode, int level, int type);
void gpio_high(int pin);
void gpio_low(int pin);
void pwm_init(int channel, int hz, int duty);
void pwm_set_duty(int channel, int duty);
void encoder_quad_init(int timer, int a, int b);
void uart_write_buffer(int uart, const uint8 *data, uint32 size);
void uart_rx_interrupt(int uart, uint32 enabled);
uint8 uart_query_byte(int uart, uint8 *data);
#endif
