#include "xsens.h"
#include "stm32f427xx.h"
#include <stdio.h>
#include "delay.h"
#include "pin.h"
#include "state.h"
#include "enet.h"

#define PORTF_UART_RX_PIN     6
#define PORTF_UART_TX_PIN     7
#define PORTF_UART_TXEN_PIN   8
#define PORTF_UART_DUPLEX_PIN 9

//  { GPIOF, GPIOF, GPIOF, GPIOF, 6,  7,  8,  9, 8, UART7  }

typedef enum { XSENS_PRE, XSENS_BID, XSENS_MID, 
               XSENS_LEN, XSENS_DATA, XSENS_CS } xsens_rx_state_t;
static xsens_rx_state_t g_xsens_rx_state = XSENS_PRE;
static uint8_t g_xsens_rx_mid = 0;
static uint8_t g_xsens_rx_expected_len = 0;
static uint8_t g_xsens_rx_data[256] = {0};
static uint8_t g_xsens_rx_write_pos = 0;

void xsens_rx_byte(const uint8_t byte);
void xsens_rx_packet();
void xsens_tx_pkt(const uint8_t mid, const uint8_t len, const uint8_t *data);
void xsens_set_config_mode();
void xsens_set_measurement_mode();
void xsens_reset();

#define XSENS_RX_RING_LEN 512
static volatile uint8_t  g_xsens_rx_ring[XSENS_RX_RING_LEN];
static volatile uint16_t g_xsens_rx_ring_read_pos  = 0;
static volatile uint16_t g_xsens_rx_ring_write_pos = 0;


static void xsens_set_baudrate(const uint32_t baudrate)
{
  uint32_t brr = 0;
  if (baudrate == 1000000)
    brr = (((uint16_t)2) << 4) | 10;  // fractional divider: 10/16 for 2.625
  else if (baudrate == 115200)
    brr = (((uint16_t)22) << 4) | 12; // fractional div: 12/16 for 22.75
  else if (baudrate == 460800)
    brr = (((uint16_t)5) << 4) | 11; // fractional div: 11/16 for 5.6875
  else
  {
    printf("unknown baudrate: %d\n", (int)baudrate);
    return;
  }
  UART7->CR1 &= ~USART_CR1_UE; // disable it
  UART7->BRR = brr;
  UART7->CR1 |=  USART_CR1_UE; // re-enable it
}

void uart7_vector()
{
  volatile uint8_t __attribute__((unused)) sr = UART7->SR; // clear overruns
  volatile uint8_t byte = UART7->DR;
  g_xsens_rx_ring[g_xsens_rx_ring_write_pos] = byte;
  if (++g_xsens_rx_ring_write_pos >= XSENS_RX_RING_LEN)
    g_xsens_rx_ring_write_pos = 0;
}

void xsens_parse_rx_ring()
{
  while (g_xsens_rx_ring_read_pos != g_xsens_rx_ring_write_pos)
  {
    xsens_rx_byte(g_xsens_rx_ring[g_xsens_rx_ring_read_pos]);
    if (++g_xsens_rx_ring_read_pos >= XSENS_RX_RING_LEN)
      g_xsens_rx_ring_read_pos = 0;
  }
}

void xsens_init()
{
  printf("xsens init\r\n");
  RCC->APB1ENR |= RCC_APB1ENR_UART7EN;

  pin_set_alternate_function(GPIOF, PORTF_UART_TX_PIN, 8);
  pin_set_alternate_function(GPIOF, PORTF_UART_RX_PIN, 8);
  pin_set_output(GPIOF, PORTF_UART_TXEN_PIN, 1);
  pin_set_output(GPIOF, PORTF_UART_DUPLEX_PIN, 0); // full-duplex mode

  UART7->CR1 &= ~USART_CR1_UE;
  UART7->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  UART7->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE;
  NVIC_SetPriority(UART7_IRQn, 1);
  NVIC_EnableIRQ(UART7_IRQn);

  /*
  xsens_set_baudrate(115200); 

  //uart_set_baudrate(XSENS_UART_IDX, 115200); // default baud rate at power-up
  uint8_t baudrate = 0; // xsens codeword for 460800 baud
  xsens_tx_pkt(0x18, 1, &baudrate);
  delay_ms(1);
  */

  xsens_set_baudrate(460800); // go faster yo
  //xsens_reset();
  /*

  delay_ms(10);
  xsens_set_config_mode();
  delay_ms(10);
  uint8_t mode_bits[2];
  mode_bits[0] = 0x00; // none of the high-order stuff
  mode_bits[1] = 0x06; // send calibrated sensors and orientation
  xsens_tx_pkt(0xd0, 2, mode_bits);
  uint8_t period_bits[2];
  // 115200 / 400 = 288
  period_bits[0] = 288 / 256;
  period_bits[1] = (uint16_t)288 & 0xff; 
  xsens_tx_pkt(0x04, 2, period_bits);
  delay_ms(10);
  xsens_set_measurement_mode();
  */

  //uart_set_rx_callback(XSENS_UART_IDX, xsens_rx_byte);
}

void xsens_set_config_mode()
{
  delay_ms(10);
  xsens_tx_pkt(0x30, 0, NULL);
  delay_ms(10);
}

void xsens_set_measurement_mode()
{
  delay_ms(10);
  xsens_tx_pkt(0x16, 0, NULL);
  delay_ms(10);
}

void xsens_reset()
{
  delay_ms(1);
  xsens_tx_pkt(0, 0, NULL);
  delay_ms(10);
}

void xsens_tx_pkt(const uint8_t mid, const uint8_t len, const uint8_t *data)
{
  uint8_t pkt[256] = {0};
  pkt[0] = 0xfa; // preamble
  pkt[1] = 0xff; // bus ID
  pkt[2] = mid;
  pkt[3] = len;
  uint8_t csum = 0xff + mid + len;
  for (int i = 0; i < len; i++)
  {
    pkt[4+i] = data[i];
    csum += data[i];
  }
  pkt[4+len] = (~csum) + 1;
  printf("xsens tx: ");
  for (int i = 0; i < 4+len+1; i++)
    printf("0x%02x ", (unsigned)pkt[i]);
  printf("\r\n");
  //uart_tx(XSENS_UART_IDX, pkt, 4+len+1);
  const uint32_t data_len = 4 + len + 1;
  for (uint32_t i = 0; i < data_len; i++)
  {
    while (!(UART7->SR & USART_SR_TXE)) { } // spin until TX buffer is clear
    UART7->DR = pkt[i];
  }
  while (!(UART7->SR & USART_SR_TC)) { } // wait for TX to finish
}

void xsens_rx_byte(const uint8_t b)
{
  //printf("0x%02x\r\n", byte);
  switch (g_xsens_rx_state)
  {
    case XSENS_PRE:
      if (b == 0xfa)
        g_xsens_rx_state = XSENS_BID;
      break;
    case XSENS_BID:
      if (b == 0xff)
        g_xsens_rx_state = XSENS_MID;
      else // woah there partner. this device only is supposed to use BID 0xff
        g_xsens_rx_state = XSENS_PRE;
      break;
    case XSENS_MID:
      g_xsens_rx_mid = b;
      g_xsens_rx_state = XSENS_LEN;
      break;
    case XSENS_LEN:
      g_xsens_rx_expected_len = b;
      g_xsens_rx_write_pos = 0;
      if (g_xsens_rx_expected_len && g_xsens_rx_expected_len != 0xff)
        g_xsens_rx_state = XSENS_DATA;
      else
        g_xsens_rx_state = XSENS_CS;
      break;
    case XSENS_DATA:
      g_xsens_rx_data[g_xsens_rx_write_pos] = b;
      if (++g_xsens_rx_write_pos >= g_xsens_rx_expected_len)
        g_xsens_rx_state = XSENS_CS;
      break;
    case XSENS_CS:
    {
      uint8_t csum = 0xff + g_xsens_rx_mid + g_xsens_rx_write_pos;
      for (int i = 0; i < g_xsens_rx_write_pos; i++) 
        csum += g_xsens_rx_data[i];
      csum += b;
      if (!csum)
        xsens_rx_packet(); // save some indents...
      g_xsens_rx_state = XSENS_PRE;
      break;
    }
    default:
      g_xsens_rx_state = XSENS_PRE; // reset
      break;
  }
}

float xsens_ntoh_float(const uint8_t *p)
{
  float swap;
  uint8_t *pswap = (uint8_t *)&swap;
  pswap[0] = p[3];
  pswap[1] = p[2];
  pswap[2] = p[1];
  pswap[3] = p[0];
  //swap[4] = { p[3], p[2], p[1], p[0] };
  /*
  uint32_t swap = (((uint32_t)p[0]) << 24) |
                  (((uint32_t)p[1]) << 16) |
                  (((uint32_t)p[2]) <<  8) |
                  (((uint32_t)p[3]) <<  0);
  */
  return swap; //*((float *)&swap);
}

uint16_t xsens_ntohs(const uint8_t *p)
{
  return (p[0] << 8) | p[1]; //((s & 0xff) << 8) |
                             //((s >> 8) & 0xff);
}

void xsens_rx_packet()
{
  const uint8_t mid = g_xsens_rx_mid;
  const uint8_t payload_len = g_xsens_rx_write_pos;
  const uint8_t *payload = g_xsens_rx_data;
  //printf("xsens rx mid %d len %d\r\n", mid, payload_len);
  if (mid == 0x32 && payload_len == 54) // it's in default mode. parse it.
  {
    float a0 = xsens_ntoh_float(&payload[ 0]);
    float a1 = xsens_ntoh_float(&payload[ 4]);
    float a2 = xsens_ntoh_float(&payload[ 8]);
    float g0 = xsens_ntoh_float(&payload[12]);
    float g1 = xsens_ntoh_float(&payload[16]);
    float g2 = xsens_ntoh_float(&payload[20]);
    float m0 = xsens_ntoh_float(&payload[24]);
    float m1 = xsens_ntoh_float(&payload[28]);
    float m2 = xsens_ntoh_float(&payload[32]);
    float q0 = xsens_ntoh_float(&payload[36]);
    float q1 = xsens_ntoh_float(&payload[40]);
    float q2 = xsens_ntoh_float(&payload[44]);
    float q3 = xsens_ntoh_float(&payload[48]);
    uint16_t sample_counter = xsens_ntohs(&payload[52]);
    __disable_irq();
    g_state.accels[0]      = a0;
    g_state.accels[1]      = a1;
    g_state.accels[2]      = a2;
    g_state.gyros[0]       = g0;
    g_state.gyros[1]       = g1;
    g_state.gyros[2]       = g2;
    g_state.mags[0]        = m0;
    g_state.mags[1]        = m1;
    g_state.mags[2]        = m2;
    g_state.quaternion[0]  = q0;
    g_state.quaternion[1]  = q1;
    g_state.quaternion[2]  = q2;
    g_state.quaternion[3]  = q3;
    g_state.imu_sample_counter = sample_counter;
    __enable_irq();
    enet_tx_state();
    //printf("%+6.3f   %+6.3f   %+6.3f   %+6.3f   %8d\r\n",
    //       q0, q1, q2, q3, (int)sample_counter);
  }
}

