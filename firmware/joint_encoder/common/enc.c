#include "enc.h"
#include "pin.h"
#include <stdio.h>
#include "parity.h"
#include "stm32f411xe.h"
#include "delay.h"

// PA4 = CS
// PA5 = SCLK
// PA6 = MISO
// PA7 = MOSI
// PA3 = PWM
// PA8 = A (quadrature) = TIM1 CH1 (af1) 
// PA9 = B (quadrature) = TIM1 CH2 (af1)

// this is SPI1, using AF5 on these pins:
#define PORTA_ENC_CS   4
#define PORTA_ENC_SCLK 5
#define PORTA_ENC_MISO 6
#define PORTA_ENC_MOSI 7

#define PORTA_ENC_A 8
#define PORTA_ENC_B 9

static uint16_t enc_txrx(const uint16_t txd);

void enc_init()
{
  printf("enc_init()\r\n");  
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

/*
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  pin_set_alternate_function(GPIOA, PORTA_ENC_A, 1); // TIM1 = AF1
  //pin_set_alternate_function(GPIOA, PORTA_ENC_B, 1); // TIM1 = AF1

  TIM1->PSC = 9; // divide inbound clock by 10
  TIM1->CCMR1 |= TIM_CCMR1_CC1S_0 | // select capture mode on TI1
                 (3 << 4); // set filter=3, meaning, N=8
              
  TIM1->CCER   = TIM_CCER_CC1P; // falling edge capture  
  TIM1->CCER  |= TIM_CCER_CC1E; // enable capture

  TIM1->SMCR |= (5 << 4) | // filtered timer input 1 as sync trigger
                4; // reset mode: trigger input resets counter
  TIM1->CR1  |= TIM_CR1_CEN; // turn on counter
*/

  pin_set_output(GPIOA, PORTA_ENC_CS, 1); 
  pin_set_alternate_function(GPIOA, PORTA_ENC_SCLK, 5);
  pin_set_alternate_function(GPIOA, PORTA_ENC_MISO, 5);
  pin_set_alternate_function(GPIOA, PORTA_ENC_MOSI, 5);
  pin_set_output_speed(GPIOA, PORTA_ENC_CS  , 3); // max beef
  pin_set_output_speed(GPIOA, PORTA_ENC_SCLK, 3); // max beef
  pin_set_output_speed(GPIOA, PORTA_ENC_MISO, 3); // max beef
  pin_set_output_speed(GPIOA, PORTA_ENC_MOSI, 3); // max beef
  SPI1->CR1 = SPI_CR1_SSM  | // software slave select management
              SPI_CR1_SSI  | // assert software select state
              SPI_CR1_CPHA | // clock phase: cpol=0, cpha=1
              SPI_CR1_MSTR | // master mode
              SPI_CR1_BR_1 | // baud rate = pclk / 8 = 48/8 = 6 mhz
              SPI_CR1_DFF  | // set to 16-bit data frames
              SPI_CR1_SPE  ; // spi enable
  // todo: if we want to be fancy, set up a RX interrupt...

  /*
  enc_txrx(0x0019); // write reg 0x19 (settings2);
  enc_txrx(5 << 5); // set A/B/I res to 200
  */
}

/*
uint16_t enc_get_last_vel_cap()
{
  return TIM1->CCR1;
}
*/

static uint16_t enc_txrx(const uint16_t txd)
{
  pin_set_output_low(GPIOA, PORTA_ENC_CS);
  volatile uint16_t rxd __attribute__((unused)) = SPI1->DR; // be sure we've flushed the RX register
  SPI1->DR = txd | (g_parity_lookup[txd & 0x7fff] << 15);
  while (!(SPI1->SR & SPI_SR_RXNE)) { } // wait for it...
  delay_ns(1); // just burn a few cycles...
  pin_set_output_high(GPIOA, PORTA_ENC_CS);
  return SPI1->DR;
}

uint16_t enc_read_reg(const uint16_t reg_addr)
{
  // this is not the fastest way to do it... requires 2 SPI transfers
  enc_txrx(reg_addr);
  return enc_txrx(0);
}

void enc_print_regs()
{
  const uint16_t reg_addrs[6] = { 0x1, 0x3, 0x3ffc, 0x3ffd, 0x3ffe, 0x3fff };
  for (int i = 0; i < 6; i++)
  {
    printf("reg 0x%04x = 0x%04x\r\n", 
           reg_addrs[i], 
           enc_read_reg(0x4000 | reg_addrs[i]));
  }
}

uint16_t enc_poll_angle()
{
  return enc_read_reg(0x7fff) & 0x3fff;
}

uint32_t enc_poll_angles()
{
  enc_txrx(0x7fff); // start txrx of compensated angle
  uint32_t comp = enc_txrx(0x7ffe); // start txrx of uncomp angle
  uint32_t uncomp = enc_txrx(0); // rx uncomp angle
  return ((uncomp & 0x3fff) << 16) | (comp & 0x3fff);
}

