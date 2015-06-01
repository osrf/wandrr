#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <stdint.h>

void fpga_spi_init();
void fpga_spi_tx(const uint8_t *data, const uint16_t data_len);

#endif

