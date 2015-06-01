#include "power.h"
#include "pin.h"

// PORTD0 through PORTD5 are on/off lines for channels 0-5.

void power_init()
{
  for (int i = 0; i < 6; i++)
    pin_set_output(GPIOD, i, 0);
}

void power_set(uint8_t channel, bool on)
{
  if (channel >= 6)
    return; // adios amigo
  pin_set_output_state(GPIOD, channel, on ? 1 : 0);
}

uint8_t power_get_state()
{
  return GPIOD->IDR & 0x3f;
}
