#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "usb.h"
#include "stm32f411xe.h"
#include <string.h>
#include "enc.h"
#include <math.h>

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
  if (len < 4)
    return;
  const uint8_t cmd = data[0];
  const uint8_t subcmd = data[1];
  if (cmd == 2 && subcmd == 1) // fpga configure command
  {
    //if (!fpga_configure(0))
    //  return;
    memset(g_tx_buf, 0, 64);
    g_tx_buf[0] = 2;
    g_tx_buf[1] = 1;
    usb_tx_if_possible();
  }
}

float g_angle[3] = {0};
uint32_t g_t_angle = 0;
float g_vel[3] = {0};
uint16_t g_raw_angle = 0;
volatile uint16_t g_num_samp = 0;

void stuff_tx_buf()
{
  //uint32_t t = SYSTIME;
  //uint32_t angles = enc_poll_angles();
  //uint16_t vel_cap = enc_get_last_vel_cap();
  g_tx_buf[0] = 0; // 0 = status report
  memcpy(&g_tx_buf[4], &g_t_angle, 4);
  //memcpy(&g_tx_buf[8], &angles, sizeof(angles));
  //memcpy(&g_tx_buf[12], &vel_cap, sizeof(vel_cap));
  //g_tx_buf[16] = (GPIOA->IDR >> 8) & 0x1;
  for (int i = 0; i < 3; i++)
  {
    *((float *)(&g_tx_buf[ 8 + i*8])) = g_angle[i];
    *((float *)(&g_tx_buf[12 + i*8])) = g_vel[i];
  }
  //memcpy(&g_tx_buf[8], &g_angle, 4);
  //memcpy(&g_tx_buf[12], &g_vel, 4);
  memcpy(&g_tx_buf[32], &g_raw_angle, 2);
  memcpy(&g_tx_buf[34], (void *)&g_num_samp, 2);
  g_num_samp = 0;
  /*
  g_tx_buf[8] = angle & 0xff;
  g_tx_buf[9] = (angle >> 8) & 0xff;
  angle = enc_poll_angle();
  g_tx_buf[10] = angle & 0xff;
  g_tx_buf[11] = (angle >> 8) & 0xff;
  */
}

static uint32_t g_led_tx_count = 0;

void usb_ep1_tx_complete()
{
  stuff_tx_buf();
  usb_tx(1, g_tx_buf, sizeof(g_tx_buf));
  if (++g_led_tx_count >= 1000) 
  {
    led_toggle();
    g_led_tx_count = 0;
  }
}

int main()
{
  led_init();
  led_on();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  enc_init();
  usb_init();
  //enc_print_regs();

  printf("entering blink loop...\r\n");
  __enable_irq();
  usb_tx(1, g_tx_buf, sizeof(g_tx_buf));
  uint16_t raw_angle = 0, prev_raw_angle = 0;
  float raw_vel = 0;
  float filt_vel[3] = {0};
  float filt_angle[3] = {0};
  //float raw_vel = 0, filt_vel = 0, filt_angle = 0;
  bool filter_init = false;
  float unwrapped_raw = 0, prev_unwrapped_raw = 0;
  uint32_t t = 0;

  const float pos_gain[3] = { 0.9f, 0.99f, 0.999f };
  const float vel_gain[3] = { 0.99f, 0.999f, 0.9999f };

  int wraps = 0;

  while (1) 
  { 
    raw_angle = enc_poll_angle();
    t = SYSTIME;

    if (filter_init)
    {
      int diff = raw_angle - prev_raw_angle;
      if (diff > 8000)
        wraps--;
      else if (diff < -8000)
        wraps++;
      unwrapped_raw = (float)raw_angle + wraps * 16384;
      // calculate raw_vel in ticks/usec for numerical stability
      // TODO: use a better timebase, since we're polling @ 100 khz so there
      // is extreme quantization on the microsecond clock
      float dt_usecs = (float)(t - g_t_angle) * 1000000.0f;
      if (dt_usecs < 1.0f)
        dt_usecs = 1.0f;

      // todo: this leads to bad numerical stability after lots of wraps
      // need to re-work this crap
      raw_vel = (unwrapped_raw - prev_unwrapped_raw) / dt_usecs;
      for (int i = 0; i < 3; i++)
      {
        filt_angle[i] =         pos_gain[i]  * filt_angle[i] + 
                        (1.0f - pos_gain[i]) * unwrapped_raw;
        filt_vel[i]   =         vel_gain[i]  * filt_vel[i]   + 
                        (1.0f - vel_gain[i]) * raw_vel * 1000000.0f;
      }
    }
    else
    {
      filter_init = true;
      for (int i = 0; i < 3; i++)
      {
        filt_angle[i] = raw_angle;
        filt_vel[i] = 0;
      }
    }
    prev_raw_angle = raw_angle;
    prev_unwrapped_raw = unwrapped_raw;

    __disable_irq();
    g_t_angle = t;
    g_raw_angle = raw_angle;
    for (int i = 0; i < 3; i++)
    {
      g_angle[i] = filt_angle[i];
      g_vel[i] = filt_vel[i]; // * 0.000001f; // convert to ticks / sec
    }
    g_num_samp++;
    __enable_irq();
    
  }
  return 0;
}

