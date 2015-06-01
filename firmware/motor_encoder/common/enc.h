#ifndef ENC_H
#define ENC_H

#include <stdint.h>

void enc_init();
void enc_print_regs();
uint16_t enc_poll_angle();
uint32_t enc_poll_angles();
//uint16_t enc_get_last_vel_cap();

#endif

