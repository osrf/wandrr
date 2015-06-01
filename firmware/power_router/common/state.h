#ifndef STATE_H
#define STATE_H

#include <stdint.h>

typedef struct
{
  uint32_t time_us;
  uint8_t  channels_active;
  uint8_t  unused[3];
  int16_t  adc[8];
  float    battery_voltage[2];
  float    battery_current[2];
  float    battery_celsius[2];
} __attribute__((packed)) state_t;

extern state_t g_state;

#endif

