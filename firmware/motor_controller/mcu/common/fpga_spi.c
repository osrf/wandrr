#include "fpga_spi.h"
#include <stdio.h>
#include "pin.h"
#include "led.h"
#include "delay.h"
#include "systime.h"

// SPI2 is slave to a SPI master running on the fpga (inbound traffic)
// PB10 = spi2 sclk on AF5
// PB14 = spi2 miso on AF5
// PB15 = spi2 mosi on AF5
// PC13 = spi2 CS (via software)

// spi3 is master to a spi slave running on the fpga (outbound traffic)
// PC15 = spi3 CS (via software)
// PB12 = spi3 SCLK
// PB5  = spi3 MOSI

#define PORTB_SSPI_SCLK 10
#define PORTB_SSPI_MISO 14
#define PORTB_SSPI_MOSI 15
#define PORTC_SSPI_CS   13

#define PORTC_MSPI_CS   15
#define PORTB_MSPI_SCLK 12
#define PORTB_MSPI_MOSI  5

static SPI_TypeDef *sspi = SPI2;
static SPI_TypeDef *mspi = SPI3;

void fpga_spi_init()
{
  printf("fpga spi init()\r\n");
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // turn on SPI2. it's on APB1 @ 24 Mhz
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; // turn on SPI3. it's on APB1 @ 24 Mhz
  
  pin_set_output_speed(GPIOB, PORTB_SSPI_SCLK, 3);
  pin_set_output_speed(GPIOB, PORTB_SSPI_MISO, 3);
  pin_set_output_speed(GPIOB, PORTB_SSPI_MOSI, 3);
  pin_set_output_speed(GPIOC, PORTC_SSPI_CS, 3);
  pin_set_alternate_function(GPIOB, PORTB_SSPI_SCLK, 5);
  pin_set_alternate_function(GPIOB, PORTB_SSPI_MISO, 5);
  pin_set_alternate_function(GPIOB, PORTB_SSPI_MOSI, 5);
  pin_set_input(GPIOC, PORTC_SSPI_CS, false);
  sspi->CR1 = SPI_CR1_SSM  | // software-select slave pin 
              SPI_CR1_SSI  | // activate the slave-select pin internally
              SPI_CR1_CPOL | // set cpol, cpha = 1,1
              SPI_CR1_CPHA |
              SPI_CR1_SPE  ; // enable SPI
  sspi->CR2 = SPI_CR2_RXNEIE; // enable rx interrupt
  NVIC_SetPriority(SPI2_IRQn, 0); // high priority, respond quickly
  NVIC_EnableIRQ(SPI2_IRQn);

  pin_set_alternate_function(GPIOB, PORTB_MSPI_SCLK, 7);
  pin_set_alternate_function(GPIOB, PORTB_MSPI_MOSI, 6);
  pin_set_output(GPIOC, PORTC_MSPI_CS, 1);
  pin_set_output_speed(GPIOB, PORTB_MSPI_SCLK, 3);
  pin_set_output_speed(GPIOB, PORTB_MSPI_MOSI, 3);
  pin_set_output_speed(GPIOC, PORTC_MSPI_CS  , 3);
  mspi->CR1 = SPI_CR1_SSM  | // software-select slave pin 
              SPI_CR1_SSI  | // activate the slave-select pin internally
              SPI_CR1_CPOL | // set cpol, cpha = 1,1
              SPI_CR1_CPHA |
              SPI_CR1_BR_1 | // baud rate = 24 mhz / 4 = 6 MHz
              SPI_CR1_MSTR | // now i am the master
              SPI_CR1_SPE  ; // enable SPI
}

void fpga_spi_tx(const uint8_t *data, const uint16_t data_len)
{
  /*
  printf("tx %d bytes:\r\n", data_len);
  for (int i = 0; i < data_len; i++)
    printf("%02x ", data[i]);
  printf("\r\n");
  */
  //volatile uint32_t t_start = SYSTIME;
  GPIOC->BSRR = (1 << PORTC_MSPI_CS) << 16; // assert CS
  delay_us(1);
  for (uint32_t i = 0; i < data_len; i++)
  {
    while (!(mspi->SR & SPI_SR_TXE)) { } // wait for tx buffer room
    mspi->DR = data[i];
  }
  while (mspi->SR & SPI_SR_BSY) { } // wait for last byte TX
  delay_us(1);
  GPIOC->BSRR = 1 << PORTC_MSPI_CS; // de-assert CS
  //volatile uint32_t t_end = SYSTIME;
  //printf("tx took %d usec\r\n", (int)(t_end - t_start));
}

