#ifndef CHAIN_H
#define CHAIN_H

#include <netinet/in.h>
#include "mc_regs.h"
#include <vector>
#include "joint_state.h"
#include "mc.h"

namespace wandrr
{

class OutboundPacket;

class Chain
{
public:
  int tx_sock_, rx_sock_;
  uint8_t appendage_idx_; // used to generate the multicast group
  sockaddr_in mcast_addr_;
  std::vector<MCRegs> regs;
  std::vector<MC> mcs_;

  Chain(const char *iface = "eth1", const uint8_t appendage_idx = 0);
  ~Chain();
  void tx(const OutboundPacket &obp);
  void resetOverheatLatch(const int hop);
  void setLED(const int hop, bool on);
  void setMotionCommand(const uint8_t  hop,
                        const uint8_t  control_mode,
                        const float    target_current,
                        const float    damping,
                        const uint32_t control_id);
  void setTXInterval(const int hop, const uint16_t rate);
  void setUSBPower(const int hop, const uint8_t power_bits);
  void setChainParameters(const int hop, const bool endpoint, 
                          const uint16_t payload_len);
  void setPWMParameters(const int hop);
  void setEnableFETs(const int hop, const bool enable);
  void setPWMDuty(const int hop, const uint16_t a, const uint16_t b, const uint16_t c);
  void setFOCTarget(const int hop, const float amps);
  void setFOCEnable(const int hop, const bool enable);
  void setFOCParameters(const int hop);
  void setFootParameters(const int hop);
  void txMCU(const int hop, const uint8_t *data, const uint8_t len);
  void txMCUFrame(const int hop, const uint8_t *data, const uint8_t len);
  void addNode();
  void setAddresses(const int hop);
  void setUSBDontRespond(const int hop, const bool dont_respond);
private:
  Chain();
  Chain(const Chain &copy);
};

}

#endif

