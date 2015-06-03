#ifndef WANDRR_H
#define WANDRR_H

#include <stdint.h>
#include <boost/function.hpp>
#include "joint_state.h"
#include "mega_state.h"
#include "chain.h"
#include <vector>
#include "outbound_packet.h"
#include "steppr_interface.h"
#include "serial_router.h"
#include "power_router.h"

namespace wandrr
{

class Wandrr
{
public:
  Wandrr();
  ~Wandrr();
  bool loadParamFile(const char *filename);

  void tx(const int chain, const OutboundPacket &obp);
  static void perish_if(bool b, const char *msg);
  void listen(const double max_seconds);
  void setLED(const int chain, const int hop, bool on);
  void setTXInterval(const int chain, const int hop, const uint16_t rate);
  void setUSBPower(const int chain, const int hop, const uint8_t power_bits);
  void setChainParameters(const int chain, const int hop, 
                          const bool endpoint, const uint16_t payload_len);
  void setAddresses(const int chain, const int hop);
  void setPWMParameters(const int chain, const int hop);
  void setEnableFETs(const int chain, const int hop, const bool enable);
  void setPWMDuty(const int chain, const int hop, const uint16_t a, const uint16_t b, const uint16_t c);
  void setFOCTarget(const int chain, const int hop, const float amps);
  void setFOCEnable(const int chain, const int hop, const bool enable);
  void setFOCParameters(const int chain, const int hop);
  void setFootParameters(const int chain, const int hop);
  void resetOverheatLatch(const int chain, const int hop);
  void setMotionCommand(const int chain, const int hop,
                        const uint8_t control_mode,
                        const float target_current,
                        const float damping,
                        const uint32_t control_id);

  void txMCU(const int chain, const int hop, const uint8_t *data, const uint8_t len);
  void txMCUFrame(const int chain, const int hop, const uint8_t *data, const uint8_t len);

  typedef boost::function<void(int chain, int hop, const JointState * const)> JointStateCallback;
  void setJointStateCallback(JointStateCallback cb) { joint_state_cb_ = cb; }

  typedef boost::function<void(int chain, int hop, 
                  const uint8_t *data, const uint16_t len)> MCURxCallback;
  void setMCURXCallback(MCURxCallback cb) { mcu_rx_cb_ = cb; }
  void setLogging(const bool on);
  void logState(const int chain, const int hop, const JointState * const js);
  bool initFromParamFile(const char *filename);
  void updateState(const int chain, const int hop, 
                   const JointState * const js,
                   const double ros_time);
  void printState();

  typedef boost::function<void(StepprMegaCmd *cmd)> RxSockCallback;
  void setRxSockCallback(RxSockCallback cb) { rx_sock_cb_ = cb; }

  typedef boost::function<void(SerialRouterState *)> SerialRouterCallback;
  void setSerialRouterCallback(SerialRouterCallback cb) { sr_cb_ = cb; }

  typedef boost::function<void(PowerRouterState *)> PowerRouterCallback;
  void setPowerRouterCallback(PowerRouterCallback cb) { pr_cb_ = cb; }

  void zeroAllFOCTargets();

public:
  JointStateCallback joint_state_cb_;
  MCURxCallback mcu_rx_cb_;
  RxSockCallback rx_sock_cb_;
  SerialRouterCallback sr_cb_;
  PowerRouterCallback pr_cb_;
  std::vector<Chain *> chains_;
  bool done_;
  bool logging_;
  MegaState ms_;
  std::vector< std::vector < FILE * > > logs_;
  int steppr_rx_sock_;
  SerialRouter *sr_;
  PowerRouter *pr_;
};

}

#endif

