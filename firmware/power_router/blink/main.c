#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"

int main()
{
  led_init();
  led_off();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  while (1) 
  { 
    delay_ms(500);
    led_toggle();
  }
  return 0;
}
