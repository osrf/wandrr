#include "discharge.h"
#include "pin.h"

// PC10 = discharge_enable

#define PORTC_DISCHARGE_EN 10


void discharge_init()
{
  pin_set_output(GPIOC, PORTC_DISCHARGE_EN, 0);
}

void discharge_enable()
{
  pin_set_output(GPIOC, PORTC_DISCHARGE_EN, 1);
}

void discharge_disable()
{
  pin_set_output(GPIOC, PORTC_DISCHARGE_EN, 0);
}
