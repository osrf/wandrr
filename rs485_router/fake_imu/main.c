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
#include "state.h"

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
  //printf("waiting for xsens boot...\r\n");
  //delay_ms(2000);
  serial_init();
  dmxl_init();
  xsens_init();
  __enable_irq();
  printf("entering main loop...\r\n");
  uint32_t t_last_tx = SYSTIME;
  uint32_t imu_sample_counter = 0;
  uint32_t t_last_led_blink = SYSTIME;
  while (1) 
  { 
    if (!dmxl_busy())
      enet_process_rx_ring();
    xsens_parse_rx_ring();
    dmxl_tick();
    uint32_t t = SYSTIME;
    if (t - t_last_tx > 2500) // 400 Hz
    {
      t_last_tx = t;
      g_state.accels[0]      = 1;
      g_state.accels[1]      = 2;
      g_state.accels[2]      = 3;
      g_state.gyros[0]       = 4;
      g_state.gyros[1]       = 5;
      g_state.gyros[2]       = 6;
      g_state.mags[0]        = 7;
      g_state.mags[1]        = 8;
      g_state.mags[2]        = 9;
      g_state.quaternion[0]  = 10;
      g_state.quaternion[1]  = 11;
      g_state.quaternion[2]  = 12;
      g_state.quaternion[3]  = 13;
      g_state.imu_sample_counter = imu_sample_counter++;
      enet_tx_state();
    }
    if (t - t_last_led_blink > 100000)
    {
      t_last_led_blink = SYSTIME;
      leds_toggle(LEDS_GREEN);
    }
  }
  return 0;
}
