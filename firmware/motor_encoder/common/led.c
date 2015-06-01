#include "led.h"
#include "pin.h"

#define PORTB_LED 9

void led_init()
{
  pin_set_output(GPIOB, PORTB_LED, 0);
}

void led_on()
{
  pin_set_output_state(GPIOB, PORTB_LED, 1);
}

void led_off()
{
  pin_set_output_state(GPIOB, PORTB_LED, 0);
}

void led_toggle()
{
  GPIOB->ODR ^= 1 << PORTB_LED;
}

