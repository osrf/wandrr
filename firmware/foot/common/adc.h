#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init();
uint16_t adc_read(const int adc_idx, const uint8_t channel);

#endif

