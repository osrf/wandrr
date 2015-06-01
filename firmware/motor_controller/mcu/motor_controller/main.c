#include "led.h"
#include "delay.h"
#include <stdio.h>
#include "console.h"
#include "systime.h"
#include "flash.h"
#include "usb.h"
#include "stm32f411xe.h"
#include <string.h>
#include "fpga.h"
#include "poe.h"
#include "fpga_spi.h"
#include "pin.h"

#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))

static uint8_t g_tx_buf[64];

void usb_tx_if_possible()
{
  if (!usb_txf_avail(1, sizeof(g_tx_buf)))
    return;
  if (!usb_tx(1, g_tx_buf, sizeof(g_tx_buf)))
    printf("error in usb_tx()\r\n");
}

#define TRANSPORT_USB 1
#define TRANSPORT_SPI 2

void process_pkt(const uint8_t *data, const uint8_t len, 
                 const int transport)
{
  const uint8_t cmd = data[0];
  const uint8_t subcmd = data[1];
  if (cmd == 2 && subcmd == 1) // fpga configure command
  {
    if (!fpga_configure(0))
      return;
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    g_tx_buf[0] = 2;
    g_tx_buf[1] = 1;
    if (transport == TRANSPORT_USB)
      usb_tx_if_possible();
    else
      fpga_spi_tx(g_tx_buf, 4);
  }
  if (cmd != 1) // flash command
    return;
  uint32_t addr = 0;
  uint8_t req_len = data[2];
  if (req_len > 32)
    req_len = 32;
  memcpy(&addr, data+4, sizeof(uint32_t));

  // prepare the outbound packet a bit (just save some repetition...)
  memset(g_tx_buf, 0, sizeof(g_tx_buf));
  g_tx_buf[0] = 1; // flash command response
  g_tx_buf[1] = subcmd; // return subcmd in packet
  memcpy(&g_tx_buf[4], &addr, sizeof(addr));

  if (subcmd == 0) // read memory
  {
    //printf("read %d bytes from 0x%08x\r\n", req_len, (unsigned)addr);
    uint32_t page_num = addr >> 8;
    uint8_t page_buf[256];
    flash_read_page(page_num, page_buf);
    uint32_t page_offset = addr & 0xff;
    if (page_offset + (uint32_t)req_len > 256)
    {
      printf("tried to perform read which crossed page boundary\n");
      return; 
    }
    g_tx_buf[2] = req_len;
    memcpy(&g_tx_buf[8], &page_buf[page_offset], req_len);
    if (transport == TRANSPORT_USB)
      usb_tx_if_possible();
    else
      fpga_spi_tx(g_tx_buf, 8 + req_len);
  }
  else if (subcmd == 1) // write flash
  {
    //printf("write %d bytes to 0x%08x\r\n", req_len, (unsigned)addr);
    flash_write(addr, &data[8], req_len);
    g_tx_buf[2] = req_len;
    if (transport == TRANSPORT_USB)
      usb_tx_if_possible();
    else
      fpga_spi_tx(g_tx_buf, 8);
  }
  else if (subcmd == 2) // erase sector
  {
    if (addr & 0xffff)
    {
      printf("tried to erase un-aligned address 0x%08x...\n", (unsigned)addr);
      return;
    }
    printf("erase sector 0x%08x\r\n", (unsigned)addr);
    flash_erase_sector(addr >> 16);
    if (transport == TRANSPORT_USB)
      usb_tx_if_possible();
    else
      fpga_spi_tx(g_tx_buf, 8);
  }
}

void usb_rx(const uint8_t ep, const uint8_t *data, const uint8_t len)
{
  if (len < 4)
    return;
  process_pkt(data, len, TRANSPORT_USB);
}

#define FPGA_SPI_RXBUF_LEN 256
static uint8_t g_fpga_spi_rxbuf[FPGA_SPI_RXBUF_LEN];
static uint16_t g_fpga_spi_rxbuf_wpos = 0;

void spi2_vector()
{
  volatile uint8_t __attribute((unused)) b = SPI2->DR;
  g_fpga_spi_rxbuf[g_fpga_spi_rxbuf_wpos] = b;
  if (g_fpga_spi_rxbuf_wpos < FPGA_SPI_RXBUF_LEN - 1)
    g_fpga_spi_rxbuf_wpos++;
}

int main()
{
  led_init();
  led_on();
  console_init();
  printf("===== APP ENTRY =====\r\n");
  systime_init();
  flash_init();
  fpga_init();
  usb_init();

  printf("configuring fpga...\r\n");
  bool config_successful = false;
  for (int attempt = 0; attempt < 2; attempt++) // try a few times
  {
    if (fpga_configure(0))
    {
      printf("configuration successful\n");
      config_successful = true;
      break;
    }
    delay_ms(50);
  }

  __enable_irq();

  if (!config_successful)
  {
    while (1) 
    { 
      delay_ms(100);
      led_toggle();
    }
  }

  // todo: if needed, use golden FPGA image (?)

  printf("entering fpga spi monitor loop...\r\n");
  fpga_spi_init();
  led_off();

#define PORTC_SSPI_CS   13
  while (1)
  {
    //int pin_state = pin_get_state(GPIOC, PORTC_SSPI_CS);
    if (0 == (GPIOC->IDR & (1 << PORTC_SSPI_CS))) //!pin_state)
    {
      SPI2->CR1 &= ~SPI_CR1_SSI;
      //led_on();
    }
    else
    {
      SPI2->CR1 |=  SPI_CR1_SSI;
      if (g_fpga_spi_rxbuf_wpos)
      {
        // we got a packet hooray
        // super simple checksum to sanity-check it...
        /*
        printf("rx %d bytes\r\n", g_fpga_spi_rxbuf_wpos);
        for (int i = 0; i < g_fpga_spi_rxbuf_wpos; i++)
          printf("%02x ", g_fpga_spi_rxbuf[i]);
        printf("\r\n");
        */

        uint8_t csum = 0;
        for (int i = 0; i < g_fpga_spi_rxbuf_wpos-1; i++)
          csum += g_fpga_spi_rxbuf[i];
        if (csum == g_fpga_spi_rxbuf[g_fpga_spi_rxbuf_wpos-1])
          process_pkt(g_fpga_spi_rxbuf, g_fpga_spi_rxbuf_wpos-1, 
                      TRANSPORT_SPI);
        g_fpga_spi_rxbuf_wpos = 0;
      }
    }
  }

  return 0;
}


