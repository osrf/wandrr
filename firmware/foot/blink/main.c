#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"

int main()
{
  leds_init();
  leds_on(LED_0);
  console_init();
  printf("===== APP ENTRY =====\n");
  systime_init();
  while (1)
  {
    delay_ms(500);
    leds_toggle(LED_0 | LED_1);
  }
  return 0;
}
