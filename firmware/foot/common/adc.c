#include "adc.h"
#include "pin.h"
#include <stdio.h>
#include "stm32f411xe.h"
#include "delay.h"

// PA4  = NSS1
// PA5  = SCLK
// PA6  = MISO
// PA7  = MOSI
// PA8  = NSS2
// PA9  = BUSY1
// PA10 = BUSY2

// use SPI1 via AF5 on these pins:
#define PORTA_NSS1   4
#define PORTA_SCLK   5
#define PORTA_MISO   6
#define PORTA_MOSI   7
#define PORTA_NSS2   8
#define PORTA_BUSY1  9
#define PORTA_BUSY2 10


void adc_init()
{
  printf("adc_init()\r\n");  
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  pin_set_output(GPIOA, PORTA_NSS1, 1); 
  pin_set_output(GPIOA, PORTA_NSS2, 1); 

  pin_set_alternate_function(GPIOA, PORTA_SCLK, 5);
  pin_set_alternate_function(GPIOA, PORTA_MISO, 5);
  pin_set_alternate_function(GPIOA, PORTA_MOSI, 5);

  /*
  pin_set_output_speed(GPIOA, PORTA_NSS1, 1);
  pin_set_output_speed(GPIOA, PORTA_NSS2, 1);
  pin_set_output_speed(GPIOA, PORTA_SCLK, 1);
  pin_set_output_speed(GPIOA, PORTA_MISO, 1);
  pin_set_output_speed(GPIOA, PORTA_MOSI, 1); 
  */

  // clock phase: CPOL=0, CPHA = 0
  SPI1->CR1 = SPI_CR1_SSM  | // software slave select management
              SPI_CR1_SSI  | // assert software select state
              SPI_CR1_MSTR | // master mode
              SPI_CR1_BR_2 | // baud rate = pclk / 32 = 48/32 = 1.5 mhz
              SPI_CR1_SPE  ; // spi enable
  // todo: if we want to be fancy, set up a RX interrupt...
  // but for now, just block.
}

uint16_t adc_read(const int adc_idx, const uint8_t channel)
{
  const uint8_t control_byte = 0x80                  |  // start bit
                               (channel & 0x7) << 4  |  // channel select
                               0x4                   |  // single-ended
                               0x2                   ;  // internal clock plz
  //volatile uint16_t rx = SPI1->DR; // be sure we've flushed the RX register
  //rx = SPI1->DR; // and again.
  pin_set_output_low(GPIOA, adc_idx ? PORTA_NSS2 : PORTA_NSS1);
  delay_us(2); // just burn a few cycles...
  SPI1->DR = control_byte;
  while (!(SPI1->SR & SPI_SR_RXNE)); // wait on SPI 
  //while (!(SPI1->SR & SPI_SR_BSY)); // wait on SPI tx last few bits
  delay_us(10); // wait a bit to let BUSY assert
  //while (!(GPIOA->IDR & (adc_idx ? PORTA_BUSY2 : PORTA_BUSY1))); // wait on ADC
  //volatile uint16_t __attribute__((unused)) rx = SPI1->DR; // flush rx buffer
  SPI1->DR; // flush rx buffer
  SPI1->DR = 0;
  while (!(SPI1->SR & SPI_SR_RXNE)); // wait on SPI tx last few bits
  volatile uint16_t rx_0 = SPI1->DR;
  SPI1->DR = 0;
  while (!(SPI1->SR & SPI_SR_RXNE)); // wait on SPI tx last few bits
  volatile uint16_t rx_1 = SPI1->DR;
  SPI1->DR = 0;
  while (!(SPI1->SR & SPI_SR_RXNE)); // wait on SPI tx last few bits
  volatile uint16_t rx_2 = SPI1->DR & 0x80; 
  delay_us(2);
  pin_set_output_high(GPIOA, adc_idx ? PORTA_NSS2 : PORTA_NSS1);
  return (rx_0 << 9) | (rx_1 << 1) | (rx_2 >> 7);
}

