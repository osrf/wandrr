#ifndef FPGA_H
#define FPGA_H

#include <stdint.h>
#include <stdbool.h>

//void fpga_txrx_state();

void fpga_init();
bool fpga_configure(uint32_t flash_start_addr);

#endif

