#include "leds.h"
#include "pin.h"
#include "stm32f427xx.h"

#define PORTB_YELLOW 0
#define PORTB_GREEN  1

void leds_init()
{
  pin_set_output(GPIOB, PORTB_YELLOW, 0);
  pin_set_output(GPIOB, PORTB_GREEN, 0);
}

void leds_on(const int color)
{
  if (color == LEDS_GREEN)
    pin_set_output_state(GPIOB, PORTB_GREEN, 1);
  else if (color == LEDS_YELLOW)
    pin_set_output_state(GPIOB, PORTB_YELLOW, 1);
}

void leds_off(const int color)
{
  if (color == LEDS_GREEN)
    pin_set_output_state(GPIOB, PORTB_GREEN, 0);
  else if (color == LEDS_YELLOW)
    pin_set_output_state(GPIOB, PORTB_YELLOW, 0);
}

void leds_toggle(const int color)
{
  if (color == LEDS_GREEN)
    GPIOB->ODR ^= 1 << PORTB_GREEN;
  else if (color == LEDS_YELLOW)
    GPIOB->ODR ^= 1 << PORTB_YELLOW;
}

