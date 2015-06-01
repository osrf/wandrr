#include "console.h"
#include "stm32f411xe.h"
#include "pin.h"

// pin connections
// PA2 = usart2 TX on AF7
// PA3 = usart2 RX on AF7

#define PORTA_TX_PIN 2
#define PORTA_RX_PIN 3

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART2;

void console_init()
{
  s_console_init_complete = 1;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  pin_set_alternate_function(GPIOA, PORTA_RX_PIN, 7);
  pin_set_alternate_function(GPIOA, PORTA_TX_PIN, 7);
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  // we want 1 megabit. do this with mantissa=1 and fraction (sixteenths)=8
  s_console_usart->BRR  = (((uint16_t)1) << 4) | 8;
  s_console_usart->CR1 |=  USART_CR1_UE;
}

void console_send_block(const uint8_t *buf, uint32_t len)
{
  if (!s_console_init_complete)
    console_init();
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(s_console_usart->SR & USART_SR_TXE)) { } // wait for tx buffer to clear
    s_console_usart->DR = buf[i];
  }
  while (!(s_console_usart->SR & USART_SR_TC)) { } // wait for TX to finish
}

