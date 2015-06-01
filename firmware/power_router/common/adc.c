#include "adc.h"
#include <stdio.h>
#include "pin.h"
#include "delay.h"

// hardware connections:
// PA8  = CS
// PA9  = RESET
// PA10 = CONVST
// PB10 = SPI2 SCLK = AF5
// PC7  = MISO
// PC8  = FIRSTDATA
// PC9  = BUSY

#define PORTA_CS         8
#define PORTA_RESET      9
#define PORTA_CONVST    10
#define PORTB_SCLK      10
#define PORTC_MISO       7
#define PORTC_FIRSTDATA  8
#define PORTC_BUSY       9

// this was a mistake. the ADC MISO signal isn't hooked up to a hardware
// MISO on the MCU. So we'll bit-bang it. Which is probably fine since
// it was probably going to be a blocking call to read from the ADC anyway.

void adc_init()
{
  printf("adc_init()\r\n");

  pin_set_output(GPIOA, PORTA_RESET, 0); 
  pin_set_output(GPIOA, PORTA_CS, 1);
  pin_set_output(GPIOA, PORTA_CONVST, 1); // active-low
  pin_set_output(GPIOB, PORTB_SCLK, 1);

  pin_set_output_speed(GPIOA, PORTA_RESET, 3);
  pin_set_output_speed(GPIOA, PORTA_CS, 3); // max beef
  pin_set_output_speed(GPIOA, PORTA_CONVST, 3);
  pin_set_output_speed(GPIOB, PORTB_SCLK, 3);

  pin_set_input(GPIOC, PORTC_MISO, false);

  pin_set_output_state(GPIOA, PORTA_RESET, 1);
  delay_us(10);
  pin_set_output_state(GPIOA, PORTA_RESET, 0);
  delay_us(10);
}

void adc_read(int16_t *data)
{
  // send a CONVST pulse
  pin_set_output_state(GPIOA, PORTA_CONVST, 0);
  delay_us(1);
  pin_set_output_state(GPIOA, PORTA_CONVST, 1);
  delay_us(25);
  pin_set_output_state(GPIOA, PORTA_CS, 0);
  for (int i = 0; i < 8; i++)
    data[i] = 0;
  /*
  pin_set_output_state(GPIOB, PORTB_SCLK, 0);
  delay_us(2);
  pin_set_output_state(GPIOB, PORTB_SCLK, 1);
  delay_us(2);
  */
  for (int i = 0; i < 8; i++)
  {
    volatile int32_t adc_val = 0;
    for (int bit = 0; bit < 18; bit++)
    {
      GPIOB->BSRR = 1 << (PORTB_SCLK + 16); // set it low
      //pin_set_output_state(GPIOB, PORTB_SCLK, 0);
      //delay_us(15);
      adc_val <<= 1;
      adc_val |= ((GPIOC->IDR & (1 << PORTC_MISO)) ? 1 : 0);
      //pin_set_output_state(GPIOB, PORTB_SCLK, 1);
      GPIOB->BSRR = 1 << PORTB_SCLK; // set it high again
      //delay_us(15);
    }
    adc_val <<= 14;
    adc_val >>= 16; // arithmetic right shift
    data[i] = (int16_t)adc_val;
  }
  delay_us(2);
  pin_set_output_state(GPIOA, PORTA_CS, 1);
}

