#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../smartcarmain/user/src/motor.c"

volatile uint32 g_sys_tick;
float imu_gyro_z_dps_filter;
static uint32 irq_mask;
static int duties[2];
static uint8 wire[128];
static uint32 wire_size;
static uint8 received_byte;
static uint8 received_ready;
void uart_rx_interrupt(int uart,uint32 enabled) { assert(uart==1 && enabled==1); }
uint8 uart_query_byte(int uart,uint8 *data)
{
    assert(uart==1);
    if (!received_ready) return 0;
    *data=received_byte; received_ready=0; return 1;
}
static void feed(const char *s,int process_each_byte)
{
    while (*s)
    {
        received_byte=(uint8)*s++; received_ready=1;
        motor_speed_test_uart_rx();
        if (process_each_byte) motor_speed_test_process_rx();
    }
    motor_speed_test_process_rx();
}
uint32 __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32 mask) { irq_mask = mask; }
void gpio_init(int p,int m,int l,int t) { (void)p;(void)m;(void)l;(void)t; }
void gpio_high(int p) { (void)p; }
void gpio_low(int p) { (void)p; }
void pwm_init(int p,int h,int d) { (void)h; duties[p]=d; }
void pwm_set_duty(int p,int d) { duties[p]=d; }
void encoder_quad_init(int t,int a,int b) { (void)t;(void)a;(void)b; }
void uart_write_buffer(int uart,const uint8 *data,uint32 size)
{
    assert(uart == DEBUG_UART_INDEX);
    assert(irq_mask == 0); // Never block the control interrupt while sending.
    assert(wire_size + size <= sizeof(wire));
    memcpy(wire + wire_size, data, size);
    wire_size += size;
}

int main(void)
{
    float frame[9];
    int saved_run_base = run_base_speed;
    motor_speed_test_tick();
    assert(!motor_auto_is_running() && duties[0]==0 && duties[1]==0);

    // Encoder conversion remains signed and uses the existing calibration.
    motor_speedl=-10000; motor_speedr=10000;
    get_motor_speed();
    assert(real_speedl>0 && fabsf(real_speedl-real_speedr)<0.0001f);

    // KEY4 uses these unchanged start/is_running/stop entry points.
    g_sys_tick=10;
    motor_auto_start();
    assert(motor_auto_is_running() && irq_mask==0);
    assert(run_base_speed==saved_run_base);
    assert(target_speedl==150 && target_speedr==150);
    assert(!auto_run_requested && !joystick_control_active);
    real_speedl=120; real_speedr=130;
    yaw_rate_ref_dps=180; imu_gyro_z_dps_filter=-400;
    motor_speed_test_tick();
    assert(target_speedl==150 && target_speedr==150);
    assert(duties[0]>0 && duties[1]>0);
    assert(duties[0]<=5000 && duties[1]<=5000);
    motor_speed_test_send_vofa();
    assert(wire_size==40);
    memcpy(frame,wire,sizeof(frame));
    assert(frame[0]==120 && frame[1]==130 && frame[2]==150);
    assert(frame[3]==duties[0] && frame[4]==duties[1]);
    assert(frame[5]==Kp && frame[6]==Ki && frame[7]==Kd && frame[8]==0);
    assert(wire[36]==0 && wire[37]==0 && wire[38]==0x80 && wire[39]==0x7f);
    motor_speed_test_send_vofa();
    assert(wire_size==40); // 20 ms rate limit.
    motor_joystick_stop();
    assert(!motor_auto_is_running() && duties[0]==0 && duties[1]==0);
    assert(target_speedl==0 && target_speedr==0 && irq_mask==0);

    // Timeout is enforced in TIM6 even if the main loop is stalled.
    motor_auto_start();
    g_sys_tick=5009;
    motor_speed_test_tick();
    assert(motor_auto_is_running());
    g_sys_tick=5010;
    motor_speed_test_tick();
    assert(!motor_auto_is_running() && duties[0]==0 && duties[1]==0);
    assert(speed_test_channels[2]==0 && speed_test_channels[3]==0);

    // Unsigned timeout also works across the system-tick rollover.
    g_sys_tick=UINT32_MAX-20;
    motor_auto_start();
    g_sys_tick+=5000;
    motor_speed_test_tick();
    assert(!motor_auto_is_running());
    irq_mask=1;
    motor_auto_start();
    assert(irq_mask==1);
    motor_joystick_stop();
    assert(irq_mask==1);
    irq_mask=0;
    motor_speed_test_uart_init();
    feed("PID 8 0.4 0.02\r\n",0);
    assert(Kp==8 && Ki==0.4f && Kd==0.02f && speed_test_command_status==1);
    assert(!motor_auto_is_running()); // A parameter command must never start.
    feed("PID 9 0.5",1);
    assert(Kp==8 && Ki==0.4f); // No partial application.
    feed(" 0.01\n",1);
    assert(Kp==9 && Ki==0.5f && Kd==0.01f);
    const char *invalid[]={"PID 8 1\n","PID 8 1 2 3\n","PID 1+2+3\n",
        "PID -1 0 0\n","PID 101 0 0\n","PID 1 21 0\n","PID 1 0 21\n",
        "PID nan 1 1\n","PID 1 inf 1\n","PID 1 1 1e100\n","P\n","PI\n","PID\n"};
    for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++)
    {
        feed(invalid[i],1);
        assert(Kp==9 && Ki==0.5f && Kd==0.01f && speed_test_command_status<0);
    }
    motor_auto_start();
    motor_speed_test_tick();
    float old_effort=control_effortl;
    uint32 old_deadline=speed_test_start_tick;
    feed("  pid\t10 0.6 3e-2\r",1);
    assert(Kp==10 && Ki==0.6f && Kd==0.03f);
    assert(motor_auto_is_running() && speed_test_start_tick==old_deadline);
    assert(control_effortl==old_effort);
    char long_line[160];
    memset(long_line,' ',sizeof(long_line)-1); long_line[159]=0;
    feed(long_line,1); feed("PID 1 2 3\n",1);
    assert(Kp==10 && speed_test_command_status==-3);
    feed(long_line,0); // Overflow the ISR queue before main-loop draining.
    assert(speed_test_command_status==-3);
    feed("\nPID 7 0.3 0.01\n",1);
    assert(Kp==7 && Ki==0.3f && Kd==0.01f);
    feed("PID 2",1); g_sys_tick+=500;
    motor_speed_test_process_rx();
    assert(speed_test_command_status==-4 && Kp==7);
    feed("PID 6 0.2 0\n",1);
    assert(Kp==6 && Ki==0.2f && Kd==0);
    wire_size=0; g_sys_tick+=10;
    motor_speed_test_send_vofa(); memcpy(frame,wire,sizeof(frame));
    assert(wire_size==40 && frame[5]==6 && frame[6]==0.2f && frame[7]==0 && frame[8]==1);
    puts("PASS: idle, encoder conversion, equal targets, unchanged PID limit, start/stop, timeout, tick wrap, atomic snapshot, JustFloat, send rate");
    puts("PASS: PID line parsing, CR/LF/CRLF, chunking, range/NaN/Inf rejection, atomic update, no start/deadline reset, line/RX overflow recovery, timeout, PID echo channels");
    return 0;
}
