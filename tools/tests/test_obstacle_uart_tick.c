/* Production RX ring/parser, with hardware/command owners stubbed. */
#include <stdint.h>
#include <string.h>
#include <ctype.h>
volatile uint32_t checks,failures,first_failure;
#define CHECK(c) do{checks++;if(!(c)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
#define ZDT_UART_RX_RING_SIZE 128U
#define ZDT_UART_COMMAND_MAX_LENGTH 63U
#define __DMB() __asm volatile("dmb" ::: "memory")
static volatile uint8_t zdt_uart_rx_ring[ZDT_UART_RX_RING_SIZE],zdt_uart_rx_head,zdt_uart_rx_tail;
static volatile uint32_t zdt_uart_rx_tick[ZDT_UART_RX_RING_SIZE],zdt_uart_rx_dropped;
static char zdt_uart_command[64];static uint8_t zdt_uart_command_length,zdt_uart_command_overflow,zdt_uart_ignore_lf,zdt_uart_echo_enabled,zdt_uart_command_tick_valid;
static uint32_t zdt_uart_input_tick,zdt_uart_command_received_tick,now,last_received,calls;
static struct{uint32_t uart_rx_bytes;}zdt_status;
static uint32_t HAL_GetTick(void){return now;}
static uint8_t BT_Button_ProcessByte(uint8_t b){(void)b;return 0;}
static void ZDT_UART_ServiceHardware(void){}
static void ZDT_Print_Prompt(void){}
static void ZDT_Console_Print(const char*s){(void)s;}
static void ZDT_Console_WriteBytes(uint8_t*b,uint8_t n){(void)b;(void)n;}
static uint8_t ZDT_Match_Command(const char*c,uint32_t tick){(void)c;last_received=tick;calls++;return 1;}
#include "vobs_uart_under_test.inc"
static void push(const char*s){while(*s)ZDT_UART_RingPush((uint8_t)*s++);}
int main(void){
 now=1000;push("vobs 21 1 500 0 50\r\n");now=1500;ZDT_UART_ProcessInput();
 CHECK(calls==1);CHECK(last_received==1000);CHECK(!zdt_uart_command_tick_valid);
 now=2000;push("&OK\n");now=2400;ZDT_UART_ProcessInput();CHECK(calls==2 && last_received==2400);
 now=3000;push("vobs 21 ");now=3100;ZDT_UART_ProcessInput();
 now=3200;push("2 500 0 50\n");now=3300;ZDT_UART_ProcessInput();CHECK(calls==3 && last_received==3000);
 now=0xfffffff0U;push("vobs 21 3 500 0 50\n");now=30;ZDT_UART_ProcessInput();
 CHECK(last_received==0xfffffff0U);CHECK(now-last_received==46);CHECK(!zdt_uart_rx_dropped);
 return 0;
}
