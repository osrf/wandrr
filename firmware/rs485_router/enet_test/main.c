#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "enet.h"

int main()
{
  leds_init();
  leds_off(LEDS_GREEN);
  leds_on(LEDS_YELLOW);
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  enet_init();
  __enable_irq();
  while (1) 
  { 
    enet_process_rx_ring();
  }
  return 0;
}
