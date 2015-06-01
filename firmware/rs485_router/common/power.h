#ifndef POWER_H
#define POWER_H

#include <stdint.h>

void power_init();
void power_set(const uint8_t channel, const uint8_t on);

#endif
