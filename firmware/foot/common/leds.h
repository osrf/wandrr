#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

#define LED_0 1
#define LED_1 2

void leds_init();
void leds_on(const int led_mask);
void leds_off(const int led_mask);
void leds_toggle(const int led_mask);

#endif

