#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include <stdint.h>

namespace wandrr
{

struct JointState
{
  uint64_t fpga_time;
  uint16_t adc[3];
  uint16_t status;
  uint32_t menc_time;
  float    menc_angle;
  float    menc_vel;
  uint16_t menc_raw;
  uint8_t  halls;
  float    motor_celsius;
  float    foc_d, foc_q;
  float    effort_d, effort_q;
  float    jenc_angle_0;
  float    jenc_vel_0;
  float    jenc_angle_1;
  float    jenc_vel_1;
  float    target_current;
  uint32_t control_id;
  uint32_t foot_time;
  int16_t foot_pressures[16];
} __attribute__((packed));

}

#endif
