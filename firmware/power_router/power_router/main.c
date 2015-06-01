#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "relays.h"
#include "remote.h"
#include "discharge.h"
#include "power.h"
#include "enet.h"
#include "adc.h"
#include "state.h"
#include "battery.h"

#define ST_IDLE            0
#define ST_PRECHARGE       1
#define ST_CLOSE_CONTACTOR 2
#define ST_STABILIZE       3
#define ST_LOGIC_ON        4
#define ST_MOTOR_SWITCH_UP 5
#define ST_MOTORS_ON       6
#define ST_OPEN_CONTACTOR  7
#define ST_DISCHARGING     8

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
  adc_init();
  battery_init();
  enet_init();
  __enable_irq();
  delay_ms(100); // let pullup work

  uint32_t t_prev_state = 0;
  uint32_t t_prev_tx = 0;
  uint32_t toggle_count = 0;
  int state = ST_IDLE;
  int16_t adc_data[8];

  while (1) 
  { 
    enet_process_rx_ring();

    const int TX_USECS = 1000;
    if (SYSTIME - t_prev_tx > TX_USECS)
    {
      if (!t_prev_tx)
        t_prev_tx = SYSTIME;
      else
        t_prev_tx += TX_USECS;

      adc_read(adc_data);
      for (int i = 0; i < 8; i++)
        g_state.adc[i] = adc_data[i];

      enet_tx_state();
      if (toggle_count++ % 100 == 0)
        led_toggle();
    }

    switch(state)
    {
      case ST_IDLE:
        if (remote_get_logic_state()) // close switch for active
        {
          t_prev_state = SYSTIME;
          state = ST_PRECHARGE;
          discharge_disable();
          relays_precharge_on();
          led_on();
        }
        if (SYSTIME % 1000 == 0)
        {
          // every millisecond, let's just make sure everything is off.
          for (int i = 0; i < 6; i++)
            power_set(i, false);
          relays_contactor_off();
          relays_precharge_off();
          discharge_disable();
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
          state = ST_LOGIC_ON;
          power_set(3, true);
          power_set(4, true);
          power_set(5, true);
        }
        break;
      case ST_LOGIC_ON:
        if (!remote_get_logic_state()) // remote-control switch opened
        {
          t_prev_state = SYSTIME;
          state = ST_OPEN_CONTACTOR;
          relays_contactor_off();
        }
        if (remote_get_motor_state())
        {
          t_prev_state = SYSTIME;
          state = ST_MOTOR_SWITCH_UP;
        }
        break;
      case ST_MOTOR_SWITCH_UP:
        if (SYSTIME - t_prev_state > 100000) // debounce
        {
          if (remote_get_motor_state())
          {
            t_prev_state = SYSTIME;
            state = ST_MOTORS_ON;
            // stagger the hits on the power supply a bit
            power_set(0, true);
            delay_us(50);
            power_set(1, true);
            delay_us(50);
            power_set(2, true);
            printf("%u enabling motor power...\r\n", (unsigned)SYSTIME);
          }
        }
        if (!remote_get_motor_state()) // switch has bounced back OFF
        {
          state = ST_LOGIC_ON;
        }
        break;
      case ST_MOTORS_ON:
        if (!remote_get_motor_state()) // motor switch OFF. leave logic up.
        {
          state = ST_LOGIC_ON;
          power_set(0, false);
          delay_us(50);
          power_set(1, false);
          delay_us(50);
          power_set(2, false);
          printf("%u disabling motor power...\r\n", (unsigned)SYSTIME);
        }
        if (!remote_get_logic_state()) // shut everything down
        {
          t_prev_state = SYSTIME;
          state = ST_OPEN_CONTACTOR;
          relays_contactor_off();
        }
        break;
      case ST_OPEN_CONTACTOR:
        relays_contactor_off();
        if (SYSTIME - t_prev_state > 250000)
        {
          t_prev_state = SYSTIME;
          state = ST_DISCHARGING;
          discharge_enable();
          for (int i = 0; i < 6; i++)
            power_set(i, false);
          printf("%u cut all power, opening contactor\r\n", (unsigned)SYSTIME);
        }
        break;
      case ST_DISCHARGING:
        led_toggle();
        if (SYSTIME - t_prev_state > 8000000) // 8 seconds
        {
          led_off();
          discharge_disable();
          state = ST_IDLE;
          printf("%u discharge complete.\r\n", (unsigned)SYSTIME);
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
  return 0;
}
