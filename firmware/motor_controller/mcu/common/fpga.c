#include "fpga.h"
#include <stdio.h>
#include "stm32f411xe.h"
#include "delay.h"
#include "systime.h"
#include "pin.h"
#include "flash.h"

#define PORTA_FPGA_CONFIG_PIN     0
#define PORTA_FPGA_TDO_PIN        1
#define PORTA_FPGA_TMS_PIN        2
#define PORTA_FPGA_TCK_PIN        3
#define PORTB_FPGA_TDI_PIN        1
#define PORTB_FPGA_CONF_DONE_PIN 13
#define PORTA_FPGA_STATUS_PIN     8
#define PORTB_FPGA_DCLK_PIN       0
#define PORTA_FPGA_DATA0_PIN     10

void fpga_init()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
  pin_set_output(GPIOA, PORTA_FPGA_CONFIG_PIN, 1);
  /*
  pin_set_output(GPIOB, PORTB_FPGA_TDI_PIN,    1);
  pin_set_output(GPIOA, PORTA_FPGA_TCK_PIN,    0);
  pin_set_output(GPIOA, PORTA_FPGA_TMS_PIN,    1);
  pin_set_output_speed(GPIOA, PORTA_FPGA_TCK_PIN, 3); // max speed
  pin_set_output_speed(GPIOB, PORTB_FPGA_TDI_PIN, 3); // max speed
  */

  pin_set_alternate_function(GPIOB, PORTB_FPGA_DCLK_PIN, 6);  // af6 for spi5
  pin_set_alternate_function(GPIOA, PORTA_FPGA_DATA0_PIN, 6); // af6 for spi5
  pin_set_output_speed(GPIOB, PORTB_FPGA_DCLK_PIN, 3); // max speed
  pin_set_output_speed(GPIOA, PORTA_FPGA_DATA0_PIN, 3); // max speed

  // we'll leave SPI5 in CPOL=0, CPHA=0, to match Cyclone V spec
  // for SPI5, APB2 = PCLK = 48 MHz. let's divide PCLK by 4 to get to config DCLK
  SPI5->CR1 = SPI_CR1_SSM      | // software-select slave pin management
              SPI_CR1_SSI      | // activate the slave-select pin internally
              SPI_CR1_LSBFIRST | // send LSB first (as per Cyclone V spec)
              SPI_CR1_SPE      | // enable SPI
              /*SPI_CR1_BR_0     | */// baud rate: f_pclk / 4 = 12 MHz
              SPI_CR1_MSTR     ; // master mode
}

bool fpga_configure(uint32_t flash_start_addr)
{
  //printf("fpga_configure starting at addr 0x%08x\r\n", 
  //       (unsigned)flash_start_addr);
  pin_set_output_low(GPIOA, PORTA_FPGA_CONFIG_PIN);
  delay_us(10);
  pin_set_output_high(GPIOA, PORTA_FPGA_CONFIG_PIN);
  //volatile uint32_t t_start = SYSTIME;
  while (!pin_get_state(GPIOA, PORTA_FPGA_STATUS_PIN)) { }
  //volatile uint32_t t_end = SYSTIME;
  //printf("elapsed config-status time: 0x%08x\r\n", (unsigned)(t_end - t_start));
  delay_us(10); // needs to be at least 2 usec. Let's not cut it close.
  // now, stream out the configuration image
  static const uint32_t CONFIG_BYTES = 21061120 / 8; // cyclone V DS, p.55
  uint8_t page[256] = {0};
  for (uint32_t i = 0; i < CONFIG_BYTES; i++)
  {
    if (i % 256 == 0)
      flash_read_page(i >> 8, page);
    while (!(SPI5->SR & SPI_SR_TXE)) { } // wait for TX buffer room
    SPI5->DR = page[i & 0xff];
  }
  while (SPI5->SR & SPI_SR_BSY) { } // wait for last byte to finish
  for (int i = 0; i < 10; i++)
  {
    if (pin_get_state(GPIOB, PORTB_FPGA_CONF_DONE_PIN))
      break;
    delay_ms(1);
  }
  if (!pin_get_state(GPIOB, PORTB_FPGA_CONF_DONE_PIN))
  {
    printf("conf_done is low. config failed.\n");
    return false;
  }
  SPI5->DR = 0; // give it a few clocks to enter user mode
  return true;
}

#if 0
// spi to/from FPGA = spi2 using af5
#define PORTB_FPGA_SS_PIN     12
#define PORTB_FPGA_SCK_PIN    13
#define PORTB_FPGA_MISO_PIN   14
#define PORTB_FPGA_MOSI_PIN   15

// uart to/from FPGA = uart4 using af8
#define PORTC_FPGA_TX_PIN 10
#define PORTC_FPGA_RX_PIN 11

// gpio to/from FPGA
#define PORTD_FPGA_GPIO_PIN 3


static volatile uint32_t g_fpga_slow_sensor_idx = 0;

void fpga_uart_rx_byte(const uint8_t byte);

void fpga_init()
{
  printf("fpga_init\r\n");
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | 
                  RCC_AHB1ENR_GPIOCEN | 
                  RCC_AHB1ENR_GPIODEN |
                  RCC_AHB1ENR_DMA1EN;
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // turn on SPI2
  //GPIOD->MODER |= 1 << (PORTD_FPGA_GPIO_PIN * 2);
  // leave SS pin as GPIO; we'll manage it ourselves 
  GPIOB->MODER |=  1 << (PORTB_FPGA_SS_PIN * 2);  // set NSS as output pin
  GPIOB->BSRRL  =  1 <<  PORTB_FPGA_SS_PIN; // de-assert NSS
  GPIOB->MODER |= (2 << (PORTB_FPGA_SCK_PIN  * 2)) |
                  (2 << (PORTB_FPGA_MOSI_PIN * 2)) |
                  (2 << (PORTB_FPGA_MISO_PIN * 2));
  GPIOB->AFR[1] |= (5 << ((PORTB_FPGA_SCK_PIN -8) * 4));
  GPIOB->AFR[1] |= (5 << ((PORTB_FPGA_MISO_PIN-8) * 4));
  GPIOB->AFR[1] |= (5 << ((PORTB_FPGA_MOSI_PIN-8) * 4));
  GPIOB->OSPEEDR = (2 << (PORTB_FPGA_SS_PIN   * 2)) |
                   (2 << (PORTB_FPGA_MOSI_PIN * 2)) |
                   (2 << (PORTB_FPGA_SCK_PIN  * 2));
  // set SPI2 to CPOL=1, CPHA=1
  // set SPI2->CR1 BR[2:0] bits to 001
  // this gives SPI clock = APB1 clock (42 MHz) / 4 = 10.5 MHz
  SPI2->CR1 = SPI_CR1_BR_0 | 
              SPI_CR1_MSTR | // master mode
              SPI_CR1_CPOL | // cpol = 1
              SPI_CR1_CPHA | // cpha = 1
              SPI_CR1_SSM  | // software slave select management
              SPI_CR1_SSI  | // software slave select flag
              SPI_CR1_SPE;   // spi enable
  // set up DMA1 stream 4, channel 0 for SPI2 TX
  SPI2->CR2 = SPI_CR2_TXDMAEN; // enable TX DMA requests
  // we want channel 0 on stream 4, so we don't set any DMA_SxCR_CHSEL bits
  DMA1_Stream4->CR = DMA_SxCR_DIR_0 | // transfer is memory-to-peripheral
                     DMA_SxCR_MINC  | // increment memory pointer each byte
                     DMA_SxCR_TCIE;   // enable the transfer-complete interrupt
  DMA1_Stream4->PAR = (uint32_t)&SPI2->DR; // write to SPI2 data register
  NVIC_SetPriority(DMA1_Stream4_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  uart_set_rx_callback(6, fpga_uart_rx_byte);
}

void dma1_stream4_vector()
{
  // this interrupt vector is reached when a SPI2 TX DMA request is completed
  // note that the SPI peripheral itself is still sending the last byte
  // at this time, so we have to wait for it to complete.
  if (DMA1->HISR & DMA_HISR_TCIF4) // make sure this is what happened
  {
    DMA1->HIFCR = DMA_HIFCR_CTCIF4; // clear the pending bit
    while (!(SPI2->SR & SPI_SR_TXE)) { } // wait for last shift into tx unit
    while (SPI2->SR & SPI_SR_BSY) { } // wait until last tx is done
  }
  GPIOB->BSRRL = 1 <<  PORTB_FPGA_SS_PIN; // de-assert NSS
}

void fpga_txrx_state()
{
  g_state.checksum = 0;
  uint8_t *tx_data = (uint8_t *)&g_state;
  for (uint32_t i = 0; i < sizeof(g_state); i++)
    g_state.checksum += tx_data[i];

  GPIOB->BSRRH = 1 << PORTB_FPGA_SS_PIN; // pull down NSS
  __disable_irq();
  DMA1_Stream4->NDTR = sizeof(g_state);
  DMA1_Stream4->M0AR = (uint32_t)tx_data;
  DMA1_Stream4->CR |= DMA_SxCR_EN;
  __enable_irq();
}

typedef enum { RX_PREAMBLE, RX_LEN, RX_PAYLOAD,
               RX_CSUM_HI,  RX_CSUM_LO } fpga_parser_state_t;
static fpga_parser_state_t g_fpga_parser_state = RX_PREAMBLE;
static uint16_t g_fpga_parser_expected_length = 0;
#define MAX_RX_PAYLOAD 1500
static uint8_t g_fpga_parser_payload[MAX_RX_PAYLOAD];
static uint16_t g_fpga_parser_payload_write_pos = 0;
static uint16_t g_fpga_parser_rx_csum = 0;

void fpga_rx_dispatch_packet()
{
  const uint8_t *payload = g_fpga_parser_payload;
  const uint8_t pkt_id = payload[0];
  const uint8_t len = g_fpga_parser_payload_write_pos;
  printf("fpga_rx_dispatch_packet: %d bytes received\n", len);
  for (int i = 0; i < len; i++)
    printf("  %02d: 0x%02x\r\n", i, payload[i]);
  if (pkt_id == 1) // power control
  {
    const uint8_t power_port = payload[1];
    const uint8_t requested_state = payload[2];
    if (power_port < POWER_NUM_CHANNELS)
      power_set_state(power_port, requested_state);
    else if (power_port < 2*POWER_NUM_CHANNELS)  // next 6 = comms hotswap pwr
      power_comms_set_state(power_port - POWER_NUM_CHANNELS, requested_state);
  }
}

void fpga_uart_rx_byte(const uint8_t b)
{
  //printf("fpga_uart_rx_byte: 0x%02x\r\n", (int)b);
  switch (g_fpga_parser_state)
  {
    case RX_PREAMBLE:
      if (b == 0x42)
        g_fpga_parser_state = RX_LEN;
      break;
    case RX_LEN:
      g_fpga_parser_expected_length = b;
      g_fpga_parser_payload_write_pos = 0;
      if (g_fpga_parser_expected_length)
        g_fpga_parser_state = RX_PAYLOAD;
      else
        g_fpga_parser_state = RX_CSUM_HI;
      break;
    case RX_PAYLOAD:
      g_fpga_parser_payload[g_fpga_parser_payload_write_pos] = b;
      g_fpga_parser_payload_write_pos++;
      if (g_fpga_parser_payload_write_pos == g_fpga_parser_expected_length)
        g_fpga_parser_state = RX_CSUM_HI;
      else if (g_fpga_parser_payload_write_pos >= MAX_RX_PAYLOAD)
        g_fpga_parser_state = RX_PREAMBLE; // overrun. try to re-sync.
      break;
    case RX_CSUM_HI:
      g_fpga_parser_rx_csum = ((uint16_t)b) << 8;
      g_fpga_parser_state = RX_CSUM_LO;
      break;
    case RX_CSUM_LO:
    {
      g_fpga_parser_state = RX_PREAMBLE;
      g_fpga_parser_rx_csum |= b;
      uint16_t local_csum = 0;
      for (int i = 0; i < g_fpga_parser_payload_write_pos; i++)
        local_csum += g_fpga_parser_payload[i];
      if (g_fpga_parser_rx_csum == local_csum)
        fpga_rx_dispatch_packet();
#if 0
      if (g_fpga_parser_rx_csum == local_csum)
      {
        printf("valid csum for %d-byte payload\r\n", 
               g_fpga_parser_payload_write_pos);
      }
      else
      {
        printf("csum mismatch: 0x%02x != 0x%02x\r\n",
               local_csum, g_fpga_parser_rx_csum);
      }
#endif
      break;
    }
    default:
      g_fpga_parser_state = RX_PREAMBLE;
      break;
  }
}

#endif
