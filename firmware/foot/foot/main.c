#include "leds.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "usb.h"
#include "stm32f411xe.h"
#include <string.h>
#include <math.h>
#include "adc.h"

#define USB_INEP(i)  ((USB_OTG_INEndpointTypeDef *)(( uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))        
#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))

static uint8_t g_tx_buf[64] __attribute__((aligned(4)));

void usb_tx_if_possible()
{
  if (!usb_txf_avail(1, sizeof(g_tx_buf)))
    return;
  if (!usb_tx(1, g_tx_buf, sizeof(g_tx_buf)))
    printf("error in usb_tx()\r\n");
}

void usb_rx(const uint8_t ep, const uint8_t *data, const uint8_t len)
{
  // this is nonsense copied from another project... left here as placeholder
  if (len < 4)
    return;
  const uint8_t cmd = data[0];
  const uint8_t subcmd = data[1];
  if (cmd == 2 && subcmd == 1) // fpga configure command
  {
    memset(g_tx_buf, 0, 64);
    g_tx_buf[0] = 2;
    g_tx_buf[1] = 1;
    usb_tx_if_possible();
  }
}

uint32_t g_t_adc = 0;
uint16_t g_adc[16] = {0};
uint16_t g_num_samp = 0;

void stuff_tx_buf()
{
  g_tx_buf[0] = 0; // 0 = status report
  memcpy(&g_tx_buf[2], &g_num_samp, 2);
  memcpy(&g_tx_buf[4], &g_t_adc, 4);
  memcpy(&g_tx_buf[8], &g_adc, 16*2);
  g_num_samp++;
}

static uint32_t g_led_tx_count = 0;

void usb_ep1_tx_complete()
{
  stuff_tx_buf();
  usb_tx(1, g_tx_buf, sizeof(g_tx_buf));
  if (++g_led_tx_count >= 1000) 
  {
    // blink the LED slow enough to see it
    leds_toggle(LED_0);
    g_led_tx_count = 0;
  }
}

int main()
{
  leds_init();
  leds_off(LED_0 | LED_1);
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  adc_init();
  usb_init();

  __enable_irq();
  // stuff the first transmission. the rest will auto-stuff from ISR
  usb_tx(1, g_tx_buf, sizeof(g_tx_buf));
  uint32_t t = 0;
  uint16_t adc_buf[16] = {0}; 
  uint32_t led_toggle_count = 0;
  while (1) 
  { 
    if (++led_toggle_count % 100 == 0)
      leds_toggle(LED_1);
    // gather the adc values
    t = SYSTIME;
    for (int a = 0; a < 2; a++)
      for (int c = 0; c < 8; c++)
      {
        adc_buf[a*8+c] = adc_read(a, c);
        delay_us(10);
      }
    // now copy the whole batch to the outbound buffer
    __disable_irq();
    g_t_adc = t;
    memcpy(g_adc, adc_buf, 16*2);
    __enable_irq();
  }
  return 0;
}

