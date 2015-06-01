#include <stdint.h>
#include <stdio.h>
//#include "stm32f4xx.h"
#include "led.h"
#include "console.h"
#include "systime.h"
/*
#include "rs485.h"
#include "imu.h"
#include "delay.h"
#include "flash.h"
*/

void run_application()
{
  __disable_irq();
  SCB->VTOR = 0x20000; // move the vector table offset, then brace for impact
  __asm volatile("ldr r0, =0x08020000 \n"
                 "ldr sp, [r0]        \n"
                 "ldr pc, [r0, #4]    \n");
}

int main()
{
  led_init();
  console_init();
  printf("===== BL RESET =====\r\n");
  systime_init();
  led_off();
  //__enable_irq();

  uint32_t t_last_blink = SYSTIME;
  uint32_t t_start = SYSTIME;
  #define BLINK_HALF_PERIOD 50000
  #define BOOT_TIMEOUT 2000000
  led_on();
  while (1)
  {
    if (SYSTIME - t_last_blink > BLINK_HALF_PERIOD)
    {
      t_last_blink += BLINK_HALF_PERIOD;
      led_toggle();
    }
    //rs485_process_rx_ring();
    if (/*g_rs485_boot_requested ||*/
        (SYSTIME - t_start > BOOT_TIMEOUT /*&& !flash_writes_occurred()*/))
    {
      run_application();
    }
  }
  return 0;
}

void HardFault_Handler()
{
  __asm("BKPT #0\n");
  while (1) { }
}

