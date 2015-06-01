#include "console.h"
#include "stm32f427xx.h"
#include "pin.h"
#include <stdbool.h>

// pin connections
// PD8 = usart3 TX, AF7
// PD9 = usart3 RX, AF7

#define PORTD_TX_PIN 8
#define PORTD_RX_PIN 9

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART3;

void console_init()
{
  s_console_init_complete = true;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  pin_set_alternate_function(GPIOD, PORTD_RX_PIN, 7);
  pin_set_alternate_function(GPIOD, PORTD_TX_PIN, 7);
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  s_console_usart->BRR  = (((uint16_t)2) << 4) | 10;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->SR & USART_SR_TXE)) { } // wait for tx buffer 
    s_console_usart->DR = buf[i];
  }
  while (!(s_console_usart->SR & USART_SR_TC)) { } // wait for TX to finish
}

