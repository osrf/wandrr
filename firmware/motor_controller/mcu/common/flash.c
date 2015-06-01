#include <stdio.h>
#include "flash.h"
#include "console.h"
#include "pin.h"
#include "delay.h"

// flash chip connections:
//  PA4 = spi1 cs   = use GPIO
//  PA5 = spi1 sclk = af5
//  PA6 = spi1 miso = af5
//  PA7 = spi1 mosi = af5

#define PORTA_FLASH_CS_PIN   4
#define PORTA_FLASH_SCLK_PIN 5
#define PORTA_FLASH_MISO_PIN 6
#define PORTA_FLASH_MOSI_PIN 7

void flash_init()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  pin_set_output(GPIOA, PORTA_FLASH_CS_PIN, 1);
  pin_set_alternate_function(GPIOA, PORTA_FLASH_SCLK_PIN, 5);
  pin_set_alternate_function(GPIOA, PORTA_FLASH_MISO_PIN, 5);
  pin_set_alternate_function(GPIOA, PORTA_FLASH_MOSI_PIN, 5);
  // increase speed on the porta pins driving this interface
  GPIOA->OSPEEDR |= (1 << (PORTA_FLASH_CS_PIN   * 2)) |
                    (1 << (PORTA_FLASH_SCLK_PIN * 2)) |
                    (1 << (PORTA_FLASH_MISO_PIN * 2)) |
                    (1 << (PORTA_FLASH_MOSI_PIN * 2)) ;
  // set up the SPI interface. use max clock freq of 48/2 = 24 MHz
  SPI1->CR1 |= /*SPI_CR1_BR_0 | // spi clock = pclk (48 mhz) / 16 = 5.25 mhz
               SPI_CR1_BR_1 |*/
               SPI_CR1_MSTR | // master mode
               SPI_CR1_CPOL | // cpol=1
               SPI_CR1_CPHA | // cpha=1
               SPI_CR1_SSM  | // software slave select pin management
               SPI_CR1_SSI  | // assert software SSI pin
               SPI_CR1_SPE  ; // SPI enable
  uint8_t flash_id[20];
  flash_read_id(flash_id);
  printf("flash id: 0x");
  for (int i = 0; i < sizeof(flash_id); i++)
    printf("%02x", (unsigned)flash_id[i]);
  printf("\n");
}

void flash_spi_txrx(const uint8_t instr, const uint16_t data_len, 
                    const uint8_t *tx_data, uint8_t *rx_data,
                    const uint8_t rx_ignore_bytes)
                    __attribute__((section ("ramfunc")));

void flash_spi_txrx(const uint8_t instr, const uint16_t data_len, 
                    const uint8_t *tx_data, uint8_t *rx_data,
                    const uint8_t rx_ignore_bytes)
{
  volatile uint8_t garbage __attribute__((unused)) = SPI1->DR; // drain...
  garbage = SPI1->DR;
  GPIOA->BSRR = (1 << PORTA_FLASH_CS_PIN) << 16; // assert CS
  delay_us(1);
  while (!(SPI1->SR & SPI_SR_TXE)) { }
  SPI1->DR = instr;
  while (!(SPI1->SR & SPI_SR_TXE)) { }
  while (!(SPI1->SR & SPI_SR_RXNE)) { }
  garbage = SPI1->DR; // drain garbage rx
  for (uint32_t i = 0; i < data_len; i++)
  {
    while (!(SPI1->SR & SPI_SR_TXE)) { } // wait for tx buffer room
    SPI1->DR = tx_data ? tx_data[i] : 0; // start txrx
    while (!(SPI1->SR & SPI_SR_RXNE)) { } // wait for rx byte
    if (rx_data && i >= rx_ignore_bytes)
      rx_data[i - rx_ignore_bytes] = SPI1->DR;
    else
      garbage = SPI1->DR; // drain garbage rx
  }
  GPIOA->BSRR = 1 << PORTA_FLASH_CS_PIN; // de-assert CS
}

uint8_t flash_read_status_register()
{
  uint8_t rx_data = 0;
  flash_spi_txrx(0x05, 1, NULL, &rx_data, 0);
  return rx_data;
}

void flash_read_id(uint8_t *id_data)
{
  flash_spi_txrx(0x9e, 20, NULL, id_data, 0);
}

static uint8_t g_flash_read_page_tx_data[256+3];

void flash_read_page(const uint32_t page_num, uint8_t *page_data)
{ 
  // assume 256-byte read pages
  //uint8_t tx_data[256+3] = {0};
  g_flash_read_page_tx_data[0] = (page_num >> 8) & 0xff; // page number MSB
  g_flash_read_page_tx_data[1] = page_num & 0xff; // page number LSB
  g_flash_read_page_tx_data[2] = 0; // page-aligned reads
  flash_spi_txrx(0x03, 256+3, 
                 g_flash_read_page_tx_data, page_data, 3); // Read Data instruction
  /*
  if (page_num == 32768)
  {
    printf("read page %d...\r\n", page_num);
    for (int i = 0; i < 256; i++) 
    { 
      printf("0x%02x  ", page_data[i]); 
      if (i % 8 == 7) 
        printf("\r\n"); 
    }
  }
  */
}

void flash_write_page(const uint32_t page_num, const uint8_t *page_data)
{ 
  /*
  printf("writing page %d...\r\n", page_num);
  for (int i = 0; i < 256; i++) 
  { 
    printf("0x%02x  ", page_data[i]); 
    if (i % 8 == 7) 
      printf("\r\n"); 
  }
  */
  flash_spi_txrx(0x06, 0, NULL, NULL, 0); // send Write Enable instruction
  uint8_t tx_data[256+3] = {0};
  tx_data[0] = (page_num >> 8) & 0xff; // page number MSB
  tx_data[1] = page_num & 0xff; // page number LSB
  tx_data[2] = 0; // page-aligned writes
  for (int i = 0; i < 256; i++)
    tx_data[i+3] = page_data[i];
  flash_spi_txrx(0x02, 256+3, tx_data, NULL, 0); // Page Program instruction
  // check Write In Progress bit periodicially
  // todo: figure out a decent timeout
  int check_count = 0;
  while (flash_read_status_register() & 0x01) 
  {
    check_count++;
    for (volatile int i = 0; i < 100; i++) { } // burn some cycles
  }
  //printf("write needed %d checks\r\n", check_count);
}

void flash_write(const uint32_t addr, const uint8_t *data, const uint16_t len)
{ 
  // ensure we won't wrap around the current page
  if (len > 256 || ((addr & 0xff) + len > 256))
  {
    printf("invalid write: addr = 0x%08x len = %d\r\n", (unsigned)addr, len);
    return;
  }
  /*
  printf("writing %d bytes to 0x%08x...\r\n", len, (unsigned)addr);
  for (int i = 0; i < len; i++) 
  { 
    printf("0x%02x  ", data[i]); 
    if (i % 8 == 7) 
      printf("\r\n"); 
  }
  */
  flash_spi_txrx(0x06, 0, NULL, NULL, 0); // send Write Enable instruction
  uint8_t tx_data[256+3] = {0}; // allocate for max size (full page)
  tx_data[0] = (addr >> 16) & 0xff; // addr byte 2 (MSB)
  tx_data[1] = (addr >>  8) & 0xff; // addr byte 1
  tx_data[2] =  addr        & 0xff; // addr byte 0 (LSB)
  for (int i = 0; i < len; i++)
    tx_data[i+3] = data[i];
  flash_spi_txrx(0x02, len+3, tx_data, NULL, 0); // Page Program instruction
  // check Write In Progress bit periodicially
  // todo: figure out a decent timeout
  int check_count = 0;
  while (flash_read_status_register() & 0x01) 
  {
    check_count++;
    for (volatile int i = 0; i < 100; i++) { } // burn some cycles
  }
}

void flash_erase_sector(const uint32_t sector_num)
{
  //printf("erasing sector of page %d...\r\n", page_num);
  if (sector_num >= 256)
  {
    printf("sector number too high: %d\n", (unsigned)sector_num);
    return;
  }
  flash_spi_txrx(0x06, 0, NULL, NULL, 0); // send Write Enable instruction
  uint8_t tx_data[3] = {0};
  tx_data[0] = sector_num;
  tx_data[1] = 0;
  tx_data[2] = 0; // page-aligned writes
  flash_spi_txrx(0xd8, 3, tx_data, NULL, 0); // Sector Erase instruction
  // check Write In Progress bit periodicially
  // todo: figure out a decent timeout
  int check_count = 0;
  while (flash_read_status_register() & 0x01) 
  {
    check_count++;
    for (volatile int i = 0; i < 100; i++) { } // burn some cycles
  }
  //printf("sector erase needed %d checks\r\n", check_count);
}

void flash_print_page(const uint32_t page_num)
{
  uint8_t page_buf[256];
  flash_read_page(page_num, page_buf);
  for (int i = 0; i < 256; i += 8)
  {
    printf("%04x%02x:  ", (unsigned)page_num, (unsigned)i);
    for (int j = 0; j < 8; j++)
      printf("%02x ", page_buf[i + j]);
    printf("\n");
  }
}

#if 0
void flash_cmd_f(uint8_t sub_cmd, uint8_t *data, uint16_t data_len)
{
#if 0
  // only useful for initial/extreme debugging...
  printf("flash_cmd_f subcmd: %c  data %d:\r\n", sub_cmd, data_len);
  for (int i = 0; i < data_len; i++)
    printf("  %2d: %02x\r\n", i, data[i]);
#endif

  if (data_len < 4)
  {
    printf("need at least 4 bytes of data for the page index\r\n");
    return;
  }
  const uint32_t page_num = *((uint32_t *)data);
  data += 4;

  if (sub_cmd == 'r') // read page
  {
    //printf("reading page %d from flash...\r\n", (int)page_num);
    uint8_t page_data[256] = {0};
    flash_read_page(page_num, page_data);
    for (int i = 0; i < 256; i++)
      printf("%02x", page_data[i]);
    printf("\r\n");
  }
  else if (sub_cmd == 'e') // erase sector containing the requested page 
  {
    flash_erase_sector(page_num);
    printf("%04x\r\n", (int)page_num);
  }
  else if (sub_cmd == 'w') // write page
  {
    if (data_len < 256)
    {
      printf("need at least 256 bytes of data to write a page. (saw %d)\r\n", 
             (int)data_len);
      return;
    }
    flash_write_page(page_num, data);
    printf("%04x\r\n", (int)page_num);
  }
}
#endif

