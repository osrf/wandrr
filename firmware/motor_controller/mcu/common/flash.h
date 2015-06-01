#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

// flash page = 256 bytes
void flash_init();
void flash_read_page(const uint32_t page_num, uint8_t *page_data); 
void flash_write_page(const uint32_t page_num, const uint8_t *page_data);
void flash_write(const uint32_t addr, const uint8_t *data, const uint16_t len);
void flash_erase_sector(const uint32_t sector_num);
void flash_read_id(uint8_t *flash_id);
void flash_console_cmd(uint8_t subcmd, uint8_t *data, uint16_t data_len);
void flash_print_page(const uint32_t page_num);

#endif

