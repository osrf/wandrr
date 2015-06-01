#ifndef POWER_H
#define POWER_H

#include <stdint.h>
#include <stdbool.h>

void power_init();
void power_set(uint8_t channel, bool on);
uint8_t power_get_state();

#endif

