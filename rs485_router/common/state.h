#ifndef STATE_H
#define STATE_H

#include <stdint.h>

#define NUM_DMXL 12

typedef struct
{
  uint32_t time_us;
  float    accels[3];
  float    gyros[3];
  float    mags[3];
  float    quaternion[4];
  uint16_t imu_sample_counter;
  float    dmxl_angle[NUM_DMXL];
  float    dmxl_vel[NUM_DMXL];
  float    dmxl_load[NUM_DMXL];
  float    dmxl_voltage[NUM_DMXL];
  float    dmxl_temp[NUM_DMXL];
  uint8_t  dmxl_status[NUM_DMXL];
} __attribute__((packed)) state_t;

extern state_t g_state;

#endif

