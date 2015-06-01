#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"

int main()
{
  leds_init();
  leds_off(LEDS_GREEN);
  leds_on(LEDS_YELLOW);
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  while (1) 
  { 
    delay_ms(500);
    leds_toggle(LEDS_GREEN);
    leds_toggle(LEDS_YELLOW);
  }
  return 0;
}
