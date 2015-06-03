#ifndef MC_REGS_H
#define MC_REGS_H

namespace wandrr
{

struct MCRegs
{
  uint32_t master;               // 0
  uint32_t aux;                  // 1
  uint16_t pwm_ctrl;             // 2 
  uint16_t pwm_w_top;
  uint16_t pwm_duty_a;           // 3
  uint16_t pwm_duty_b;
  uint16_t pwm_duty_c;           // 4
  uint16_t pwm_dead_time;
  /////// FOC REGISTERS BEGIN
  float    foc_target;           // 5
  float    foc_kp;               // 6
  float    foc_ki;               // 7
  float    foc_max_effort;       // 8
  float    foc_integrator_limit; // 9
  uint16_t foc_stator_offset;    // 10
  uint8_t  foc_pole_pairs;
  uint8_t  foc_unused;
  float    foc_bus_voltage;      // 11: ureg_foc6 
  float    foc_control_id;       // 12: ureg_foc7
  /////// NETWORK REGISTERS
  uint8_t  net_endpoint;         // 13
  uint16_t net_payload_len;
  uint8_t  net_unused;
  uint32_t net_src_ip;           // 14 ureg_net1
  uint32_t net_dst_ip;           // 15 ureg_net2
  uint8_t  net_dst_mac[6];       // 16, 17 ureg_net3, LSB's of 4
  uint8_t  net_src_mac[6];       // 17, 18 ureg_net4 MSB's, ureg_net5
  uint32_t net_unused2;          // 19 ureg_net6
  uint32_t net_unused3;          // 20 ureg_net7
  //////// more FOC registers
  float    foc_damping;          // 21 ureg_foc8
  float    foc_resistance;       // 22 ureg_foc9
  float    foc_temperature_limit;// 23 ureg_foc10
  float    foc_unused3;          // 24 ureg_foc11
  //////// more random stuff
  uint8_t  foot_control;         // 25 foot
  uint8_t  foot_usb_port;
  uint16_t foot_unused;
} __attribute__((packed));

}

#endif

