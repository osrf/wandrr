#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "flash.h"
#include "usb.h"
#include "stm32f411xe.h"

#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))

int main()
{
  led_init();
  led_on();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  flash_init();
  usb_init();
  /*
  printf("about to print page 0...\r\n");
  flash_print_page(0);
  printf("hooray hooray\r\n");
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
  __enable_irq();
  uint8_t test_msg[64] = {0};
  while (1) 
  { 
    delay_ms(500);
    led_toggle();
    /*
    printf("intsts = %08x doepctl2 = %08x\r\n", 
           (unsigned)USB_OTG_FS->GINTSTS,
           (unsigned)USB_OUTEP(2)->DOEPCTL);
    */
    if (usb_txf_avail(1, sizeof(test_msg)))
    {
      printf("tx\r\n");
      if (!usb_tx(1, test_msg, sizeof(test_msg)))
        printf("error in usb_tx()\r\n");
    }
    else
    {
      printf("unable to tx\r\n");
    }
  }
  return 0;
}

