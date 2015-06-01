#include "stm32f427xx.h"
#include "serial.h"
#include "pin.h"
#include "delay.h"

#define NUM_PORTS 4

typedef struct
{
  GPIO_TypeDef *rx_gpio, *tx_gpio, *txe_gpio, *duplex_gpio;
  uint8_t rx_pin, tx_pin, txe_pin, duplex_pin, af;
  USART_TypeDef *uart;
} serial_port_t;

static serial_port_t g_ports[NUM_PORTS] = 
{
  { GPIOD, GPIOD, GPIOD, GPIOD, 6,  5,  2,  3, 7, USART2 },
  { GPIOG, GPIOG, GPIOG, GPIOG, 9, 14, 10, 11, 8, USART6 },
  { GPIOB, GPIOB, GPIOG, GPIOB, 7,  6, 13,  5, 7, USART1 },
  { GPIOE, GPIOE, GPIOE, GPIOE, 0,  1,  2,  3, 8, UART8  }
};

void serial_init()
{
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
  for (int i = 0; i < NUM_PORTS; i++)
  {
    serial_port_t *p = &g_ports[i];
    pin_set_alternate_function(p->tx_gpio, p->tx_pin, p->af);
    pin_set_alternate_function(p->rx_gpio, p->rx_pin, p->af);
    pin_set_output(p->txe_gpio, p->txe_pin, 0);
    int duplex = (i == 4 ? 0 : 1); // last port is full-duplex, others half
    pin_set_output(p->duplex_gpio, p->duplex_pin, duplex); 
    USART_TypeDef *u = p->uart;
    if (u == USART6 || u == USART1)
      u->BRR = (((uint16_t)5) << 4) |  4; // divisor of 5.25  for 1M baud
    else
      u->BRR = (((uint16_t)2) << 4) | 10; // divisor of 2.625 for 1M baud
    u->CR1 |= USART_CR1_UE; // enable the uart
  }
}

void serial_tx(const uint8_t port, const uint8_t *data, const uint8_t len)
{
  if (port >= NUM_PORTS)
    return; // adios amigo
  serial_port_t *p = &g_ports[port];
  USART_TypeDef *u = p->uart;
  u->CR1 &= ~USART_CR1_RE; // disable receiver during transmit
  //u->CR1 |= USART_CR1_TE; // enable transmitter
  pin_set_output_state(p->txe_gpio, p->txe_pin, 1);
  delay_us(5);
  for (int i = 0; i < len; i++)
  {
    while (!(u->SR & USART_SR_TXE)) { } // spin for tx buffer to clear
    u->DR = data[i];
  }
  while (!(u->SR & USART_SR_TC)) { } // spin waiting for completion
  delay_us(5);
  pin_set_output_state(p->txe_gpio, p->txe_pin, 0); // de-assert TXE
  u->CR1 |=  USART_CR1_RE; // re-enable receiver
  //u->CR1 &= ~USART_CR1_TE; // disable transmitter
}

