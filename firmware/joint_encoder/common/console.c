#include "console.h"
#include "stm32f411xe.h"
#include "pin.h"

// pin connections
// PB6 = usart1 TX on AF7
// PB7 = usart1 RX on AF7

#define PORTB_TX_PIN 6
#define PORTB_RX_PIN 7

static volatile bool s_console_init_complete = false;
static volatile USART_TypeDef * const s_console_usart = USART1;

void console_init()
{
  s_console_init_complete = 1;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  pin_set_alternate_function(GPIOB, PORTB_RX_PIN, 7);
  pin_set_alternate_function(GPIOB, PORTB_TX_PIN, 7);
  s_console_usart->CR1 &= ~USART_CR1_UE;
  s_console_usart->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  // we want 1 megabit. do this with mantissa=3 and fraction (sixteenths)=0
  s_console_usart->BRR  = (((uint16_t)3) << 4) | 0;
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

