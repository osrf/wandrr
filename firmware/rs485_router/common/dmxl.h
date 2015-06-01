#ifndef DMXL_H
#define DMXL_H

#include <stdint.h>
#include <stdbool.h>

void dmxl_init();
void dmxl_tick();
void dmxl_set_regs(const uint8_t port, const uint8_t id,
                   const uint8_t start_addr, const uint8_t len,
                   const uint8_t *regs);
void dmxl_set_goals(const uint16_t *goals);
void dmxl_set_goal(const uint8_t port, const uint8_t id, const uint16_t goal);
bool dmxl_busy();

#endif

