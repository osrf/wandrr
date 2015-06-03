#ifndef OUTBOUND_PACKET_H
#define OUTBOUND_PACKET_H

#include <stdint.h>
#include <string.h>
#include <cstdio>
#include "mc_regs.h"

namespace wandrr
{

class OutboundPacket
{
public:
  static const int BUF_LEN = 2000;
  uint8_t buf_[BUF_LEN];
  int write_pos_;
  OutboundPacket()
  {
    reset();
  }
  ~OutboundPacket() { }
  void reset()
  {
    memset(buf_, 0, BUF_LEN);
    buf_[0] = 0x21; // sentinel byte 0
    buf_[1] = 0x43; // sentinel byte 1
    buf_[2] = 0x00; // starting hop count = 0 (low byte)
    buf_[3] = 0x00; // starting hop count = 0 (high byte)
    write_pos_ = 4;
  }
  void appendSetAux(uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 8; // TODO: submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 1; // start writing at register 1 (auxiliary)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->aux, 4);
    write_pos_ += 4;
  }
  /*
  void append_set_foc(const uint8_t hop, 
                      const float current, const uint16_t stator_offset,
                      const bool enable = true)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // high byte of addr
    buf_[write_pos_++] = 4 + 8*4; // submsg len LSB
    buf_[write_pos_++] = 0; // submsg len MSB
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0:  no idea
    buf_[write_pos_++] = 0; // start writing at register 1 (master)
    buf_[write_pos_++] = 0; // unused
    MCRegs *mc_regs = (MCRegs *)&buf_[write_pos_];
    mc_regs->master = (1 << 16) | // send every-other ADC cycle
                      (enable ? 0x4 : 0x0) |  // FOC enable
                      (enable ? 0x1 : 0x0) ;  // master enable
    mc_regs->aux = 0x10000 | 0x100 | (enable ? 1 : 0); // turn on LED and usb 1
    mc_regs->pwm_ctrl = 0x1; // take input duty values from FOC 
    mc_regs->pwm_w_top = 0x1fff; // 125e6/8192 = 15259 Hz PWM frequency
    mc_regs->pwm_duty_a = 0x1000;
    mc_regs->pwm_duty_b = 0x1000;
    mc_regs->pwm_duty_c = 0x1000;
    mc_regs->pwm_dead_time = 62; // 125 = 1uS   62 = 500 nS dead time
    mc_regs->foc_stator_offset = stator_offset;
    mc_regs->foc_target = current;
    mc_regs->foc_kp = -10; //-20; // 10  //0.005; //-3;
    mc_regs->foc_ki = -0.2; //0.001; //-10;
    mc_regs->foc_max_effort = 15;
    mc_regs->foc_integrator_limit = 1000;
    mc_regs->foc_unused = 0;
    mc_regs->foc_unused2 = 0;
    mc_regs->foc_unused3 = 0;
  }
  */
  void appendSetMaster(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 8; // TODO: submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 0; // start writing at register 0 (master)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->master, 4);
    write_pos_ += 4;
  }
  void appendSetChainParameters(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 8; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 13; // start writing at register 13 (net0)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->net_endpoint, 4);
    write_pos_ += 4;
  }
  void appendSetAddresses(const int hop, const MCRegs *regs)
  {
    printf("sending setaddresses for hop %d\n", hop);
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4*5; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 14; // start writing at register 14 (net1 = src IP)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->net_src_ip, 4*5);
    write_pos_ += 4*5;
  }
  void appendSetPWMParameters(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4*3; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 2; // start writing at register 2 (pwm0)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->pwm_ctrl, 12);
    write_pos_ += 12;
  }
  void appendSetPWMDuty(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4*2; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 3; // start writing at register 3 (pwm1)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->pwm_duty_a, 8);
    write_pos_ += 8;
  }
  void appendSetFOCTarget(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 5; // start writing at register 5 (foc target)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->foc_target, 4);
    write_pos_ += 4;
  }
  void appendSetFOCParameters(const uint8_t hop, const MCRegs *regs)
  {
    // set the first batch of 8 registers
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4*8; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 5; // start writing at register 5 (foc0)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->foc_target, 4*8);
    write_pos_ += 4*8;
  }
  void appendSetExtraFOCParameters(const uint8_t hop, const MCRegs *regs)
  {
    // set the first batch of 8 registers
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4*4; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 21; // start writing at register 5 (foc8)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->foc_damping, 4*4);
    write_pos_ += 4*4;
  }
  void appendTxMCU(const uint8_t hop, const uint8_t *data, const uint8_t len)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 2 + len; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 3; // command 1: MCU comms (over SPI)
    buf_[write_pos_++] = 0; // subcommand 0: tx to MCU
    memcpy(&buf_[write_pos_], data, len);
    write_pos_ += len;
  }
  void appendSetFootParameters(const uint8_t hop, const MCRegs *regs)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 4 + 4; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 1; // command 1: write register
    buf_[write_pos_++] = 0; // subcommand 0: can't remember
    buf_[write_pos_++] = 25; // start writing at register 5 (foot)
    buf_[write_pos_++] = 0; // unused
    memcpy(&buf_[write_pos_], &regs->foot_control, 4);
    write_pos_ += 4;
  }
  void appendMotionCommand(const uint8_t  hop,
                           const uint8_t  control_mode,
                           const float    target_current,
                           const float    damping,
                           const uint32_t control_id)
  {
    buf_[write_pos_++] = hop;
    buf_[write_pos_++] = 0; // only handles 8-bit addrs now
    buf_[write_pos_++] = 15; // submsg len (low byte)
    buf_[write_pos_++] = 0; // submsg len (high byte)
    buf_[write_pos_++] = 4; // command 4: motion command
    buf_[write_pos_++] = 0; // subcommand 0: not used
    buf_[write_pos_++] = control_mode; 
    memcpy(&buf_[write_pos_], &target_current, 4);
    write_pos_ += 4;
    float damping_scaled = damping * -1.0e6;
    memcpy(&buf_[write_pos_], &damping_scaled, 4);
    write_pos_ += 4;
    memcpy(&buf_[write_pos_], &control_id, 4);
    write_pos_ += 4;
  }
};

}

#endif
