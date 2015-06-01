#include <stdint.h>
#include <stdio.h>
#include "led.h"
#include "console.h"
#include "systime.h"

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
  led_on();
  systime_init();
  console_init();
  //console_send_block((uint8_t *)"hello, world!\r\n", 13);
  puts("===== BL RESET =====\r\n");
  uint32_t t_last_blink = SYSTIME;
  uint32_t t_start = SYSTIME;
  #define BLINK_HALF_PERIOD  100000
  #define BOOT_TIMEOUT       200000

  while (1)
  {
    if (SYSTIME - t_last_blink > BLINK_HALF_PERIOD)
    {
      t_last_blink += BLINK_HALF_PERIOD;
      led_toggle();
    }
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

