#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void serial_init();
void serial_tx(const uint8_t port, const uint8_t *data, const uint8_t len);

#endif

