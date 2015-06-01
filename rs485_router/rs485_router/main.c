#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "enet.h"
#include "power.h"
#include "serial.h"
#include "dmxl.h"
#include "xsens.h"

int main()
{
  leds_init();
  leds_off(LEDS_GREEN);
  leds_on(LEDS_YELLOW);
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  enet_init();
  power_init();
  power_set(4, 1); // turn on the xsens port
  printf("waiting for xsens boot...\r\n");
  delay_ms(2000);
  serial_init();
  dmxl_init();
  xsens_init();
  __enable_irq();
  printf("entering main loop...\r\n");
  while (1) 
  { 
    if (!dmxl_busy())
      enet_process_rx_ring();
    xsens_parse_rx_ring();
    dmxl_tick();
  }
  return 0;
}
