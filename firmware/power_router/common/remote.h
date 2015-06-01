#ifndef REMOTE_H
#define REMOTE_H

#include <stdbool.h>

void remote_init();
bool remote_get_motor_state();
bool remote_get_logic_state();

#endif
