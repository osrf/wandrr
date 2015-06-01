#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "relays.h"
#include "remote.h"
#include "discharge.h"
#include "power.h"

#define ST_IDLE            0
#define ST_PRECHARGE       1
#define ST_CLOSE_CONTACTOR 2
#define ST_RUNNING         3
#define ST_STABILIZE       4
#define ST_OPEN_CONTACTOR  5
#define ST_DISCHARGING     6

int main()
{
  led_init();
  led_off();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  relays_init();
  discharge_init();
  remote_init();
  power_init();
  delay_ms(100); // let pullup work

  uint32_t t_prev_state = 0;
  int state = ST_IDLE;

  while (1) 
  { 
    switch(state)
    {
      case ST_IDLE:
        if (!remote_get_state()) // close switch for active
        {
          t_prev_state = SYSTIME;
          state = ST_PRECHARGE;
          discharge_disable();
          relays_precharge_on();
          led_on();
        }
        break;
      case ST_PRECHARGE:
        led_toggle();
        if (SYSTIME - t_prev_state > 3000000)
        {
          t_prev_state = SYSTIME;
          state = ST_CLOSE_CONTACTOR;
          led_on();
          relays_contactor_on();
        }
        break;
      case ST_CLOSE_CONTACTOR:
        if (SYSTIME - t_prev_state > 500000) 
        {
          t_prev_state = SYSTIME;
          state = ST_STABILIZE;
          relays_precharge_off();
        }
        break;
      case ST_STABILIZE:
        if (SYSTIME - t_prev_state > 1000000)
        {
          t_prev_state = SYSTIME;
          state = ST_RUNNING;
          power_set(0, true);
          power_set(1, true);
          power_set(2, true);
        }
        break;
      case ST_RUNNING:
        if (remote_get_state()) // remote-control switch opened
        {
          t_prev_state = SYSTIME;
          state = ST_OPEN_CONTACTOR;
          relays_contactor_off();
        }
        break;
      case ST_OPEN_CONTACTOR:
        if (SYSTIME - t_prev_state > 500000)
        {
          t_prev_state = SYSTIME;
          state = ST_DISCHARGING;
          discharge_enable();
        }
        break;
      case ST_DISCHARGING:
        led_toggle();
        if (SYSTIME - t_prev_state > 8000000) // 8 seconds
        {
          led_off();
          discharge_disable();
          state = ST_IDLE;
          power_set(0, false);
          power_set(1, false);
          power_set(2, false);
        }
        break;
      default: // shouldn't ever be here
        relays_precharge_off();
        relays_contactor_off();
        discharge_disable();
        state = ST_IDLE;
        break;
    }
  }
  /*
  relays_precharge_on();
  delay_ms(2000);
  relays_contactor_on();
  delay_ms(1000);
  relays_precharge_off();
  while (1) 
  { 
    delay_ms(500);
    led_toggle();
  }
  */
  return 0;
}
