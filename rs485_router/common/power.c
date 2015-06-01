#include "power.h"
#include "pin.h"
#include "leds.h"

// PD4  = on A
// PG12 = on B
// PG15 = on C
// PE4  = on D
// PF10 = on E

#define PORTD_CHA_ON 4
#define PORTG_CHB_ON 12
#define PORTG_CHC_ON 15
#define PORTE_CHD_ON 4
#define PORTF_CHE_ON 10

void power_init()
{
  pin_set_output(GPIOD, PORTD_CHA_ON, 0);
  pin_set_output(GPIOG, PORTG_CHB_ON, 0);
  pin_set_output(GPIOG, PORTG_CHC_ON, 0);
  pin_set_output(GPIOE, PORTE_CHD_ON, 0);
  pin_set_output(GPIOF, PORTF_CHE_ON, 0);
}

void power_set(const uint8_t channel, const uint8_t on)
{
  switch (channel)
  {
    case 0: pin_set_output_state(GPIOD, PORTD_CHA_ON, on); break;
    case 1: pin_set_output_state(GPIOG, PORTG_CHB_ON, on); break;
    case 2: pin_set_output_state(GPIOG, PORTG_CHC_ON, on); break;
    case 3: pin_set_output_state(GPIOE, PORTE_CHD_ON, on); break;
    case 4: pin_set_output_state(GPIOF, PORTF_CHE_ON, on); break;
  }
  if (channel == 1)
  {
    if (on)
      leds_on(LEDS_GREEN);
    else
      leds_off(LEDS_GREEN);
  }
}

