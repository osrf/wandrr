#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "adc.h"

int main()
{
  leds_init();
  leds_on(LED_0);
  console_init();
  printf("===== APP ENTRY =====\n");
  adc_init();
  systime_init();
  printf("entering main loop\n");
  uint16_t adc[16] = {0};
  while (1)
  {
    delay_ms(500);
    leds_toggle(LED_0 | LED_1);
    for (int a = 0; a < 2; a++)
      for (int c = 0; c < 8; c++)
        adc[a*8+c] = adc_read(a, c);

    for (int a = 0; a < 2; a++)
    {
      for (int c = 0; c < 8; c++)
        printf("%8d ", adc[a*8+c]);
      printf("\n");
    }
    printf("\n");
  }
  return 0;
}
