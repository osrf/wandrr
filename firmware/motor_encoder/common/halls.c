#include "halls.h"
#include "pin.h"

#define PORTA_HALL_PWR 10
#define PORTA_HALL_A    0
#define PORTA_HALL_B    1
#define PORTA_HALL_C    2

void halls_init()
{
  pin_set_output(GPIOA, PORTA_HALL_PWR, 1);
}

