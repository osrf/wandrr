#include "poe.h"
#include "pin.h"

#define PORTB_POE_POWER_CTRL 8

void poe_init()
{
  pin_set_output(GPIOB, PORTB_POE_POWER_CTRL, 0);
}

void poe_power(const bool on)
{
  pin_set_output_state(GPIOB, PORTB_POE_POWER_CTRL, on ? 1 : 0);
}
