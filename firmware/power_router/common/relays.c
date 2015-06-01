#include "relays.h"
#include "pin.h"

// PE2  = contactor enable
// PC11 = precharge enable

#define PORTE_CONTACTOR_EN 2
#define PORTC_PRECHARGE_EN 11


void relays_init()
{
  pin_set_output(GPIOE, PORTE_CONTACTOR_EN, 0);
  pin_set_output(GPIOC, PORTC_PRECHARGE_EN, 0);
}

void relays_precharge_on()
{
  pin_set_output(GPIOC, PORTC_PRECHARGE_EN, 1);
}

void relays_precharge_off()
{
  pin_set_output(GPIOC, PORTC_PRECHARGE_EN, 0);
}

void relays_contactor_on()
{
  pin_set_output(GPIOE, PORTE_CONTACTOR_EN, 1);
}

void relays_contactor_off()
{
  pin_set_output(GPIOE, PORTE_CONTACTOR_EN, 0);
}

