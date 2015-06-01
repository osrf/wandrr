#ifndef HALLS_H
#define HALLS_H

#include "stm32f411xe.h"

void halls_init();

static inline uint8_t halls_get_state()
{
  return GPIOA->IDR & 0x7; // bottom three bits are the halls
}

#endif

