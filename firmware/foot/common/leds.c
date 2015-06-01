#include "leds.h"
#include "pin.h"

#define PORTB_LED_0 12
#define PORTB_LED_1 13

void leds_init()
{
  pin_set_output(GPIOB, PORTB_LED_0, 0);
  pin_set_output(GPIOB, PORTB_LED_1, 0);
}

void leds_on(const int led_mask)
{
  if (led_mask & LED_0)
    pin_set_output_state(GPIOB, PORTB_LED_0, 1);
  if (led_mask & LED_1)
    pin_set_output_state(GPIOB, PORTB_LED_1, 1);
}

void leds_off(const int led_mask)
{
  if (led_mask & LED_0)
    pin_set_output_state(GPIOB, PORTB_LED_0, 0);
  if (led_mask & LED_1)
    pin_set_output_state(GPIOB, PORTB_LED_1, 0);
}

void leds_toggle(const int led_mask)
{
  if (led_mask & LED_0)
    GPIOB->ODR ^= 1 << PORTB_LED_0;
  if (led_mask & LED_1)
    GPIOB->ODR ^= 1 << PORTB_LED_1;
}

