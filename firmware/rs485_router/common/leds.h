#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

#define LEDS_GREEN  0
#define LEDS_YELLOW 1

void leds_init();
void leds_on(const int color);
void leds_off(const int color);
void leds_toggle(const int color);

#endif

