#include "usb.h"
#include "stm32f411xe.h"
#include <stdbool.h>
#include <stdio.h>
#include "delay.h"
#include "pin.h"
#include <string.h>

extern void usb_ep1_txf_empty() __attribute__((weak));
extern void usb_ep1_tx_complete() __attribute__((weak));

static bool g_usb_config_complete = false;
#define PORTA_USB_DM_PIN 11
#define PORTA_USB_DP_PIN 12

#define USB_TIMEOUT 200000
static USB_OTG_DeviceTypeDef * const g_usbd = 
  (USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + 
                            USB_OTG_DEVICE_BASE);
#define USB_INEP(i)  ((USB_OTG_INEndpointTypeDef *)(( uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))        
#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_WAIT_RET_BOOL(cond) \
  do { \
    int count = 0; \
    do { \
      if ( ++count > USB_TIMEOUT ) \
        return false; \
    } while (cond); \
  } while (0)
#define USB_FIFO_BASE ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE))
#define USB_FIFO(i) ((uint32_t *)(USB_FIFO_BASE + 1024 * i))

typedef struct
{
  uint8_t  length;
  uint8_t  descriptor_type;
  uint8_t  endpoint_address;
  uint8_t  attributes;
  uint16_t max_packet_size;
  uint8_t  interval;
} __attribute__((packed)) usb_ep_desc_t;

typedef struct
{
  uint8_t length;
  uint8_t descriptor_type;
  uint8_t interface_number;
  uint8_t alternate_setting;
  uint8_t num_endpoints;
  uint8_t interface_class;
  uint8_t interface_subclass;
  uint8_t interface_protocol;
  uint8_t interface_string_index;
  usb_ep_desc_t eps[2];
} __attribute__((packed)) usb_iface_desc_t;

typedef struct 
{
  uint8_t  length;
  uint8_t  descriptor_type;
  uint16_t total_length;
  uint8_t  num_interfaces;
  uint8_t  configuration_value;
  uint8_t  configuration_string_index;
  uint8_t  attributes;
  uint8_t  max_power;
  usb_iface_desc_t ifaces[1];
} __attribute__((packed)) usb_config_desc_t;

static const usb_config_desc_t g_usb_config_desc = 
{ 
  9, // length of this configuration descriptor
  2, // configuration descriptor type
  9 + 9 + 2 * 7, // total length 
  1, // number of interfaces
  0, // configuration value
  0, // no string
  0x80, // attributes
  50, // max power
  {
    {
      9, // length of this interface descriptor
      4, // interface descriptor type
      0, // interface number
      0, // alternate setting
      2, // no extra endpoints
      0xff, // custom class code,
      0xff, // custom subclass code,
      0xff, // custom protocol code
      0, // no string 
      { 
        {
          7, // length of this endpoint descriptor
          5, // endpoint descriptor type
          0x81, // EP1 IN
          0x02, // bulk endpoint
          64, // max packet size
          1 // interval. ignored for bulk endpoints anyway.
        },
        {
          7, // length of this endpoint descriptor
          5, // endpoint descriptor type
          0x02, // EP2 OUT
          0x02, // bulk endpoint
          64, // max packet size
          1 // interval. ignored for bulk endpoints anyway.
        }
      }
    }
  }
};

bool usb_flush_txfifo(uint32_t fifo)
{
  USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (fifo << 5);
  USB_WAIT_RET_BOOL((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == 
                    USB_OTG_GRSTCTL_TXFFLSH);
  return true;
}

bool usb_flush_rxfifo()
{
  USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
  USB_WAIT_RET_BOOL((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) ==
                    USB_OTG_GRSTCTL_RXFFLSH);
  return true;
}

static bool usb_reset()
{
  uint32_t i = 0;
  do
  {
    if (++i > USB_TIMEOUT)
    {
      printf("hit timeout waiting for ahb idle state\r\n");
      return false;
    }
  } while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
  USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST; // start core soft reset
  i = 0;
  do
  {
    if (++i > USB_TIMEOUT)
    {
      printf("hit timeout waiting for core reset to complete\r\n");
      return false;
    }
  } while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST); 
  return true;
}

uint8_t g_usb_device_descriptor[18];

void usb_init()
{
  g_usb_device_descriptor[0] = 18; // TODO
  g_usb_device_descriptor[1] = 1; // device desciptor type
  g_usb_device_descriptor[2] = 0x00; // usb 2.0
  g_usb_device_descriptor[3] = 0x02; // usb 2.0
  g_usb_device_descriptor[4] = 0xff; // custom device class
  g_usb_device_descriptor[5] = 0xff; // no subclass code for custom device
  g_usb_device_descriptor[6] = 0xff; // no protocol code for custom device
  g_usb_device_descriptor[7] = 64;   // max packet size for EP0
  g_usb_device_descriptor[8] = 0x55; // vid
  g_usb_device_descriptor[9] = 0xf0; // vid
  g_usb_device_descriptor[10] = 0x25; // pid
  g_usb_device_descriptor[11] = 0x01; // pid
  g_usb_device_descriptor[12] = 0x00; // bcdDevice
  g_usb_device_descriptor[13] = 0x00; // bcdDevice
  g_usb_device_descriptor[14] = 0x00; // manufacturer string idx
  g_usb_device_descriptor[15] = 0x00; // product string idx
  g_usb_device_descriptor[16] = 0x00; // serial number string idx
  g_usb_device_descriptor[17] = 1; // number of possible configurations

  pin_set_alternate_function(GPIOA, PORTA_USB_DM_PIN, 10); // usb is AF10
  pin_set_alternate_function(GPIOA, PORTA_USB_DP_PIN, 10); // usb is AF10
  pin_set_output_speed(GPIOA, PORTA_USB_DM_PIN, 3); // max beef
  pin_set_output_speed(GPIOA, PORTA_USB_DP_PIN, 3); // max beef

  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // turn on USB OTG FS clock gate
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL; // is this necessary?
  delay_ms(10);
  if (usb_reset())
    printf("usb core reset successfully\r\n");
  else
    printf("usb core failed to reset\r\n");
  //USB_OTG_FS->GINTSTS = 0; // |= USB_OTG_GINTSTS_RXFLVL; // set rx fifo level (?)
  USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN     | // wake up the transceiver (?)
                      USB_OTG_GCCFG_NOVBUSSENS ;
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD  | // set device mode
                         (10 << 10)             | // set turnaround time to 10 clk
                         USB_OTG_GUSBCFG_PHYSEL | // use built-in PHY
                         5; // TOCAL value (?)
  //delay_ms(50);
  // enable all PHY clock gates. Maybe coming out of reset this isn't needed...
  // not quite sure. But let's do it anyway.
  *((uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE)) = 0;
  g_usbd->DCFG &= ~USB_OTG_DCFG_DAD; // zero out the device address fields
  g_usbd->DCFG |= 0x3 | // peg to full-speed
                  USB_OTG_DCFG_NZLSOHSK; // non-zero length handshake (?)
  // wait for reset interrupt
  // Wait for the ENUMDNE interrupt in OTG_FS_GINTSTS. 
  // This interrupt indicates the end of reset on the USB. 
  // On receiving this interrupt, the application must read the OTG_FS_DSTS
  // USB on-the-go full-speed (OTG_FS) register to determine the 
  // enumeration speed and perform the steps listed in Endpoint
  // initialization on enumeration completion on page 770.
  USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM     | // start of frame
                         //USB_OTG_GINTMSK_ESUSPM   | // early suspend
                         //USB_OTG_GINTMSK_USBSUSPM | // usb suspend
                         USB_OTG_GINTMSK_USBRST   | // usb reset
                         USB_OTG_GINTMSK_ENUMDNEM | // enumeration done
                         //USB_OTG_GINTMSK_OTGINT   | // otg interrupt 
                         USB_OTG_GINTMSK_RXFLVLM  | // something is the RX FIFO
                         USB_OTG_GINTMSK_MMISM    | // mode mismatch
                         USB_OTG_GINTMSK_IEPINT   ; // IN endpoint interrupt

  USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT    | // enable usb interrupt
                         USB_OTG_GAHBCFG_TXFELVL ; // interrupt only when empty
                         // actually, let's leave this at half-empty for now...
  NVIC_SetPriority(OTG_FS_IRQn, 1); // high priority, respond quickly
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

extern void usb_rx(const uint8_t ep, 
                   const uint8_t *data, 
                   const uint8_t len) __attribute__((weak));

void usb_rx_internal(const uint8_t ep, const uint8_t *data, const uint8_t len)
{
  if (ep >= 4)
    return; // adios amigo
  USB_OUTEP(ep)->DOEPTSIZ = (1 << 19) | 64; // buffer one full-length packet
  USB_OUTEP(ep)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK  | // clear nak to allow more
                            USB_OTG_DOEPCTL_EPENA ; // re-enable RX endpoint
  if (usb_rx)
    usb_rx(ep, data, len);
}

bool usb_tx(const uint8_t ep, const uint8_t *payload, const uint8_t payload_len)
{
  if (ep >= 4)
    return false;
  if (ep != 0 && !g_usb_config_complete)
    return false; // can't transmit yet
  USB_INEP(ep)->DIEPTSIZ = (1 << 19) | payload_len; // set outbound txlen
  USB_INEP(ep)->DIEPCTL  |= USB_OTG_DIEPCTL_EPENA | // enable endpoint
                            USB_OTG_DIEPCTL_CNAK  ; // clear NAK bit
  uint32_t *fifo = USB_FIFO(ep); // memory-mapped magic...
  for (int word_idx = 0; word_idx < (payload_len + 3) / 4; word_idx++)
    *fifo = *((uint32_t *)&payload[word_idx * 4]); // abomination
  return true;
}

bool usb_txf_avail(const uint8_t ep, const uint8_t nbytes)
{
  if (ep >= 4)
    return false;
  return USB_INEP(ep)->DTXFSTS >= nbytes / 4;
}

bool usb_tx_stall(const uint8_t ep)
{
  if (ep >= 4)
    return false; // adios amigo
  USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  return true;
}

bool usb_setup_rx(uint8_t *buf, const uint32_t len)
{
  const uint8_t  request_type = buf[0];
  const uint8_t  request = buf[1];
  const uint16_t request_value = buf[2] | ((uint16_t)buf[3] << 8);
  const uint16_t request_len   = buf[6] | ((uint16_t)buf[7] << 8);
  //printf("setup rx type 0x%02x request 0x%02x\r\n", request_type, request);
  if (request_type == 0x0 && request == 0x5) // set address
  {
    const uint16_t addr = buf[2];
    g_usbd->DCFG |= (addr << 4);
    usb_tx(0, NULL, 0);
    printf("set addr %d\r\n", addr);
    return true;
    //return usb_tx(0, NULL, 0); // send an empty status packet back
  }
  else if (request_type == 0x80 && request == 0x0) // get status
  {
    uint8_t status[2] = { 0, 0 };
    return usb_tx(0, status, sizeof(status)); // send an empty status packet back
  }
  else if (request_type == 0x80 && request == 0x6) // get descriptor
  {
    uint8_t *pdesc = NULL;
    uint8_t desc_len = 0;
    //printf("desc req val = 0x%04x\r\n", (unsigned)request_value);
    if (request_value == 0x0100) // request device descriptor
    {
      pdesc = g_usb_device_descriptor;
      desc_len = sizeof(g_usb_device_descriptor);
    }
    else if (request_value == 0x0200) // request configuration descriptor
    {
      pdesc = (uint8_t *)&g_usb_config_desc;
      desc_len = g_usb_config_desc.total_length;
    }
    if (pdesc)
    {
      int write_len = request_len < desc_len ? request_len : desc_len;
      // todo: loop until done sending it, if needed
      return usb_tx(0, pdesc, write_len);
    }
    else
      return usb_tx_stall(0); // we don't know how to handle this request
  }
  else if (request_type == 0 && request == 0x9) // set configuration
  {
    if (request_value != 0)
      return usb_tx_stall(0);
    // enable the IN endpoint for configuration #0
    //USB_INEP(1)->DIEPTSIZ = (1 << 19) | payload_len; // set outbound pkt size
    //USB_INEP(1)->DIEPINT  = USB_OTG_DIEPINT_TXFE; // interrupt on TXF empty
    USB_INEP(1)->DIEPINT  = USB_OTG_DIEPINT_XFRC; // interrupt on TXF empty
    USB_INEP(1)->DIEPTSIZ = 64; 
    USB_INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK    | // set NAK bit
                           (1 << 22)               | // use txfifo #1
                           USB_OTG_DIEPCTL_EPTYP_1 | // bulk transfer
                           USB_OTG_DIEPCTL_USBAEP  | // active EP (always 1)
                           USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                           64; // max packet size

    // enable the OUT endpoint for configuration #0
    USB_OUTEP(2)->DOEPTSIZ = (1 << 19) | 64; // buffer one full-length packet
    USB_OUTEP(2)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                            USB_OTG_DOEPCTL_USBAEP | // active endpoint (always 1)
                            USB_OTG_DOEPCTL_CNAK   | // clear NAK bit
                            USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                            USB_OTG_DOEPCTL_EPTYP_1 |
                            64; // max packet size = 64 bytes
    g_usbd->DAINTMSK = 0x40002; // OUT2 and IN1 IRQ . used to be 0x10001
    // done configuring endpoints; now we'll send an empty status packet back
    g_usb_config_complete = true;
    uint8_t bogus[64] = {0};
    usb_tx(1, bogus, sizeof(bogus)); // kick-start the FIFO low interrupt
    return usb_tx(0, NULL, 0); 
  }
  printf("unhandled usb_setup_rx len %u\r\n  ", (unsigned)len);
  for (int i = 0; i < len; i++)
    printf("%02x ", buf[i]);
  printf("\r\n");
  return false;
}


void otg_fs_vector()
{
  //printf("otg vect\r\n");
  //printf("unhandled gintsts = %08x\r\n", (unsigned)USB_OTG_FS->GINTSTS);
  if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)
  {
    printf("usb reset\r\n");
    g_usbd->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    usb_flush_txfifo(0);
    g_usbd->DCFG |= 0x3 | // peg to full-speed
                    USB_OTG_DCFG_NZLSOHSK; // non-zero length handshake (?)
    for (int i = 0; i < 4; i++)
    {
      USB_OUTEP(i)->DOEPINT = 0xff; // wipe out any pending EP flags
      USB_INEP(i)->DIEPINT = 0xff; // wipe out any pending EP flags
      //USB_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    }
    //g_usbd->DAINT = 0xffffffff; // wipe out any flags (aren't these r/o ?)
    //g_usbd->DAINTMSK = 0x10001; // enable IN0 and OUT0 for control msgs
    g_usbd->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |  // enable setup-done irq
                       USB_OTG_DOEPMSK_EPDM  |  // enable EP-disabled irq
                       USB_OTG_DOEPMSK_XFRCM ;  // enable tx-done irq
    g_usbd->DIEPMSK |= USB_OTG_DIEPMSK_TOM   |  // timeout irq
                       USB_OTG_DIEPMSK_XFRCM |
                       USB_OTG_DIEPMSK_EPDM;
    // set up FIFO sizes, in bytes:
    #define USB_RXFIFO_SIZE     512
    #define USB_TXFIFO_EP0_SIZE 128
    #define USB_TXFIFO_EP1_SIZE 128
     
    USB_OTG_FS->GRXFSIZ = USB_RXFIFO_SIZE / 4; // size is in 32-bit words !
    g_usbd->DCFG &= ~USB_OTG_DCFG_DAD; // zero out the device address fields
    uint32_t usb_ram_addr = USB_RXFIFO_SIZE;
    // the length of this buffer is in 32-bit words, but addr is bytes. argh.
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((USB_TXFIFO_EP0_SIZE/4) << 16) |
                                     usb_ram_addr;
    usb_ram_addr += USB_TXFIFO_EP0_SIZE;
    // the length of this buffer is in 32-bit words, but addr is bytes. argh.
    USB_OTG_FS->DIEPTXF[0] = ((USB_TXFIFO_EP1_SIZE/4) << 16) | 
                             usb_ram_addr;

    USB_INEP (0)->DIEPTSIZ = 64;
    USB_OUTEP(0)->DOEPTSIZ = (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)) |
                             USB_OTG_DOEPTSIZ_STUPCNT | // allow 3 setup pkt
                             (3 * 8); // transfer size
    USB_INEP (0)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | // active EP (always 1)
                            USB_OTG_DIEPCTL_SNAK   ; // set NAK bit
    USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                            USB_OTG_DOEPCTL_USBAEP | // active EP (always 1)
                            USB_OTG_DOEPCTL_CNAK   ; // clear NAK bit

    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
  {
    //printf("enum done\r\n");
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_SOF; // clear the flag (rc_w1)
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
  {
    static uint8_t setup_buf[64] = {0};
    static uint8_t setup_nbytes = 0;
    const volatile uint32_t rx_status = USB_OTG_FS->GRXSTSP;
    const uint8_t epnum = rx_status & USB_OTG_GRXSTSP_EPNUM;
    const uint16_t nbytes = (rx_status & USB_OTG_GRXSTSP_BCNT) >> 4;
    if (nbytes > 64)
    {
      printf("woah there partner. nbytes = %d\r\n", (int)nbytes);
      return;
    }
    if (epnum >= 4)
    {
      printf("woah there partner. epnum = %d\r\n", (int)epnum);
      return;
    }

    //uint8_t dpid = (rx_status & USB_OTG_GRXSTSP_DPID) >> 15;
    const uint8_t pktsts = (rx_status & USB_OTG_GRXSTSP_PKTSTS) >> 17;
    //printf("rx epnum = %d nbytes = %d dpid = %d pktsts = %d\r\n",
    //       epnum, nbytes, dpid, pktsts);
    uint8_t buf[72] = {0}; // bigger than the largest possible usb packet...
    int wpos = 0;
    uint32_t w = 0;
    for (int rword = 0; rword < nbytes/4; rword++)
    {
      w = *USB_FIFO(epnum);
      buf[wpos++] =  w        & 0xff;
      buf[wpos++] = (w >>  8) & 0xff;
      buf[wpos++] = (w >> 16) & 0xff;
      buf[wpos++] = (w >> 24) & 0xff;
    }
    if (nbytes % 4)
    {
      w = *USB_FIFO(epnum);
      for (int partial = 0; partial < nbytes % 4; partial++)
      {
        buf[wpos++] = w & 0xff;
        w >>= 8;
      }
    }

    if (epnum == 0)
    {
      //printf("RXFLVL ep0 pktsts = %d\r\n", pktsts);
      if (pktsts == 6)
      {
        memcpy(setup_buf, buf, nbytes);
        setup_nbytes = nbytes;
      }
      else if (pktsts == 4)
      {
        usb_setup_rx(setup_buf, setup_nbytes);
        USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA  | // enable endpoint
                                USB_OTG_DOEPCTL_USBAEP | // active endpoint (?)
                                USB_OTG_DOEPCTL_CNAK   ; // clear NAK bit
      }
    }
    else 
    {
      //printf("pktsts %d nbytes %d\r\n", pktsts, nbytes);
      if (nbytes) // ignore transfer-complete notifications
        usb_rx_internal(epnum, buf, nbytes);
    }
  }
  else if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT)
  {
    //printf("iepint vec = 0x%08x\r\n", (unsigned)g_usbd->DAINT);
    //if (usb_ep1_txf_empty)
    //  usb_ep1_txf_empty();

    if (g_usbd->DAINT & 0x2)
    {
      // EP1 is firing an IRQ
      if (USB_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC)
      {
        USB_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC; // clear the flag
        // fire the handler
        if (usb_ep1_tx_complete)
          usb_ep1_tx_complete();
      }
    }
  }
  else
  {
    printf("unhandled gintsts = %08x\r\n", (unsigned)USB_OTG_FS->GINTSTS);
  }
}

