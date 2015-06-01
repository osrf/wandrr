#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "flash.h"

int main()
{
  led_init();
  led_on();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  flash_init();
  printf("about to print page 0...\r\n");
  flash_print_page(0);
  printf("hooray hooray\r\n");
  /*
  printf("erasing sector 0...\r\n");
  flash_erase_sector(0);
  flash_print_page(0);
  printf("writing page 0 with ascending sequence...\r\n");
  uint8_t page[256];
  for (int i = 0; i < 256; i++)
    page[i] = i;
  flash_write_page(0, page);
  flash_print_page(0);
  flash_print_page(1);
  */

  printf("entering blink loop...\r\n");
  while (1) 
  { 
    delay_ms(500);
    led_toggle();
  }
  return 0;
}
