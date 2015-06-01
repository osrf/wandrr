#include "remote.h"
#include "pin.h"
#include "stm32f427xx.h"
#include <stdio.h>
#include "systime.h"

// pa0 = rc tx via USART2 on AF7
// pa3 = rc rx via USART2 on AF7

// for now, just a gpio switch connected to PA3
//#define PORTA_REMOTE_MOTORS 3
//#define PORTA_REMOTE_LOGIC  0

#define PORTA_REMOTE_TX 0
#define PORTA_REMOTE_RX 3

static USART_TypeDef * const g_remote_uart = USART2;

void remote_init()
{
  printf("remote_init()\r\n");
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  pin_set_alternate_function(GPIOA, PORTA_REMOTE_RX, 7);
  pin_set_input(GPIOA, PORTA_REMOTE_TX, false); // don't need to TX yet
  g_remote_uart->CR1 &= ~USART_CR1_UE;
  g_remote_uart->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
  // baud rate calculation: 42e6 / 16 / 22.75 ~= 115200
  g_remote_uart->BRR = (((uint16_t)22) << 4) | 13; 
  g_remote_uart->CR1 |= USART_CR1_UE;
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
}

volatile uint8_t g_remote_switch_byte = 0;
volatile uint32_t g_remote_switch_rx_time = 0;

void usart2_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART2->SR; // clear overruns
  volatile uint8_t b = USART2->DR;
  //printf("rx 0x%02x\r\n", b);
  g_remote_switch_byte = b;
  g_remote_switch_rx_time = SYSTIME;
}

#define REMOTE_SWITCH_TIMEOUT_US 100000

bool remote_get_motor_state()
{
  __disable_irq();
  volatile uint32_t remote_age = SYSTIME - g_remote_switch_rx_time;
  __enable_irq();
  if (remote_age > REMOTE_SWITCH_TIMEOUT_US)
    g_remote_switch_byte = 0; // who are you. i don't even know you anymore
  return g_remote_switch_byte == 0xc3;
  //return false;
  //return pin_get_state(GPIOA, PORTA_REMOTE_MOTORS);
}

bool remote_get_logic_state()
{
  __disable_irq();
  volatile uint32_t remote_age = SYSTIME - g_remote_switch_rx_time;
  __enable_irq();
  if (remote_age > REMOTE_SWITCH_TIMEOUT_US)
    g_remote_switch_byte = 0; // who are you. i don't even know you anymore
  return g_remote_switch_byte != 0; 
  //return pin_get_state(GPIOA, PORTA_REMOTE_LOGIC);
}
