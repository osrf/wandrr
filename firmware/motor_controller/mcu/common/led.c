#include "led.h"
#include "pin.h"
#include "stm32f411xe.h"

#define PORTA_LED 9

void led_init()
{
  pin_set_output(GPIOA, PORTA_LED, 0);
}

void led_on()
{
  pin_set_output_state(GPIOA, PORTA_LED, 1);
}

void led_off()
{
  pin_set_output_state(GPIOA, PORTA_LED, 0);
}

void led_toggle()
{
  GPIOA->ODR ^= 1 << PORTA_LED;
}

