#include <ifaddrs.h>
#include "chain.h"
#include "wandrr.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
using namespace wandrr;

//#define PRINT_ETH_PACKETS

Chain::Chain(const char *iface, const uint8_t appendage_idx)
: appendage_idx_(appendage_idx)
{
  printf("opening chain for appendage_idx %d\n", appendage_idx);
  char mcast_addr_cstr[80];
  snprintf(mcast_addr_cstr, sizeof(mcast_addr_cstr), 
           "224.0.0.%d", 124 + appendage_idx_);
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  Wandrr::perish_if(tx_sock_ < 0, "couldn't create tx socket");
  Wandrr::perish_if(rx_sock_ < 0, "couldn't create rx socket");
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_addr_cstr);
  ifaddrs *ifaddr;
  Wandrr::perish_if(getifaddrs(&ifaddr) == -1, "couldn't get ipv4 iface addr");
  in_addr_t local_ipv4_addr = 0;

  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr)
      continue;
    if (ifa->ifa_addr->sa_family != AF_INET)
      continue;
    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
          host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;
    //printf("found addr %s on interface %s\n", host, ifa->ifa_name);
    if (!strcmp(ifa->ifa_name, iface))
      local_ipv4_addr = inet_addr(host);
  }
  freeifaddrs(ifaddr);
  if (!local_ipv4_addr)
  {
    printf("couldn't find interface address of %s\n", iface);
    exit(1);
  }
  in_addr local_addr;
  local_addr.s_addr = local_ipv4_addr;
  int rc = 0;
  rc = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF,
      (char *)&local_addr, sizeof(local_addr));
  Wandrr::perish_if(rc < 0, "couldn't allow multicasting for udp tx sock");
  //////////////////////////////////////////////////////////////////////////
  int loopback = 0; // no loopback, thanks
  rc = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_LOOP,
                  (char *)&loopback, sizeof(loopback));
  Wandrr::perish_if(rc < 0, "couldn't turn off multicast loopback");
  ////////////////////////////
  int reuseaddr = 1;
  rc = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
      &reuseaddr, sizeof(reuseaddr));
  Wandrr::perish_if(rc < 0, "couldn't set SO_REUSEADDR on UDP RX sock");

  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = mcast_addr_.sin_addr.s_addr; // INADDR_ANY;
  rx_bind_addr.sin_port = htons(11300);

  rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port 11300");

  printf("binding sock %d to %s\n", rx_sock_, mcast_addr_cstr);

  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_cstr);
  mreq.imr_interface.s_addr = local_ipv4_addr;
  rc = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  &mreq, sizeof(mreq));
  Wandrr::perish_if(rc < 0, "couldn't add ourselves to the multicast group");

  //rc = setsockopt(rx_sock_, SOL_SOCKET, SO_BINDTODEVICE, 
  //                iface, strlen(iface)+1);
  //Wandrr::perish_if(rc < 0, "couldn't bind to device");

}

Chain::~Chain()
{
}

void Chain::addNode()
{
  int i = (int)regs.size();
  // populate it with sane defaults
  regs.push_back(MCRegs());
  regs[i].master = (4883 << 16); // send @ 4 Hz
  regs[i].aux = 0x21000000; // set bit 16 for motor-direction reverse
  regs[i].pwm_ctrl = 1; // pull voltages from FOC
  regs[i].pwm_w_top = 5119; // 100e6/5120/2 = 9765 Hz PWM
  regs[i].pwm_duty_a = 2559;
  regs[i].pwm_duty_b = 2559;
  regs[i].pwm_duty_c = 2559;
  regs[i].pwm_dead_time = 62; // 500 nS ?
  ////////
  regs[i].foc_target = 0;
  regs[i].foc_kp = -2;
  regs[i].foc_ki = -200;
  regs[i].foc_max_effort = 35;
  regs[i].foc_integrator_limit = 20;
  regs[i].foc_stator_offset = 459;
  regs[i].foc_pole_pairs = 8; // default... but not correct in all cases
  regs[i].foc_unused = 0;
  regs[i].foc_bus_voltage = 100.0;
  regs[i].foc_control_id = 0;
  ///////
  regs[i].net_endpoint = 1;
  regs[i].net_payload_len = 200;
  regs[i].net_unused = 0;
  regs[i].net_dst_ip = 0xe000007c + appendage_idx_; // multicast group
  regs[i].net_src_ip = 0x0a42ac30 + (appendage_idx_ << 8); // source ip
  regs[i].net_dst_mac[0] = 0x7c + appendage_idx_;
  regs[i].net_dst_mac[1] = 0x00;
  regs[i].net_dst_mac[2] = 0x00;
  regs[i].net_dst_mac[3] = 0x5e;
  regs[i].net_dst_mac[4] = 0x00;
  regs[i].net_dst_mac[5] = 0x01;
  regs[i].net_src_mac[0] = 0x11 + appendage_idx_;
  regs[i].net_src_mac[1] = 0x00;
  regs[i].net_src_mac[2] = 0x00;
  regs[i].net_src_mac[3] = 0xc1;
  regs[i].net_src_mac[4] = 0xf3;
  regs[i].net_src_mac[5] = 0xa4;
  regs[i].net_unused2 = 0;
  regs[i].net_unused3 = 0;
  //////
  regs[i].foc_damping = 0;
  regs[i].foc_resistance = 0;
  regs[i].foc_temperature_limit = 90;
  regs[i].foc_unused3 = 0;
  //////
  regs[i].foot_control = 0;
  regs[i].foot_usb_port = 0;
  regs[i].foot_unused = 0;

  mcs_.push_back(MC()); 
}

void Chain::tx(const OutboundPacket &obp)
{
  const uint8_t *data = obp.buf_;
  const uint16_t len = obp.write_pos_;
  const uint16_t port = 11300;
  mcast_addr_.sin_port = htons(port);
  int nsent = sendto(tx_sock_, data, len, 0,
      (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  if (nsent != len)
    printf("woah, sendto() returned %d\n", nsent);

#ifdef PRINT_ETH_PACKETS
  uint16_t eth_len = len + 42; // headers: 14 + 20 + 8 = 42
  if (eth_len < 60)
    eth_len = 60;
  const uint16_t ipv4_len = len + 28;
  const uint16_t udp_len = len + 8;
  //printf("sending %d byte payload, %d byte total:\n", 
  //       buf_len, eth_len);
  printf("%08x\n", eth_len);
  printf("00010000\n00001000\n");
  printf("01005e00\n");
  printf("00%02x902b\n", 0x7c + appendage_idx_); // dest MAC, random src MAC
  printf("3439be2e\n");
  printf("0800"); // ipv4 ethertype
  uint16_t header[14]; // just for copy-pasting into the verilog simulator...
  header[0]  = 0x4500; // ipv4
  header[1]  = ipv4_len;
  header[2]  = 0; // id
  header[3]  = 0x4000; // flags
  header[4]  = 0x0111; // ttl and proto (UDP)
  header[5]  = 0; // to be filled in with checksum
  header[6]  = 0xc0a8; // source ip: 192.168
  header[7]  = 0x0180; // source ip: 000.128
  header[8]  = 0xe000; // mcast dest ip: 224.000
  header[9]  = 0x007c + appendage_idx_;  // mcast dest ip: 000.124+idx
  header[10] = 0xccad; // ephemeral source port
  header[11] = port; // destination port
  header[12] = udp_len;
  header[13] = 0; // udp checksum is optional in ipv4

  uint32_t header_csum = 0;
  for (int i = 0; i < 10; i++)
    header_csum += header[i];
  header[5] = ~((header_csum & 0xffff) + (header_csum >> 16));

  for (int i = 0; i < 14; i++)
  {
    printf("%04x", header[i]);
    if (i % 2 == 0)
      printf("\n");
  }

  for (int i = 0; i < len; i++)
  {
    printf("%02x", data[i]);
    if (i % 4 == 1)
      printf("\n");
  }
  printf("\n\n");
#endif
}

void Chain::resetOverheatLatch(const int hop)
{
  if (hop >= regs.size())
    return;
  regs[hop].master |= 0x8; // set the "reset overheat latch" bit ON
  OutboundPacket pkt;
  pkt.appendSetMaster(hop, &regs[hop]);
  tx(pkt);
  regs[hop].master &= ~0x8; // reset the "reset overheat latch" bit
  pkt.reset();
  pkt.appendSetMaster(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setMotionCommand(const uint8_t  hop,
                             const uint8_t  control_mode,
                             const float    target_current,
                             const float    damping,
                             const uint32_t control_id)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendMotionCommand(hop, control_mode,
                          target_current, damping, control_id);
  tx(pkt);
}

void Chain::setLED(const int hop, bool on)
{
  if (hop >= regs.size())
    return;
  if (on)
    regs[hop].aux |= 1;
  else
    regs[hop].aux &= ~1;
  OutboundPacket pkt;
  pkt.appendSetAux(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setUSBDontRespond(const int hop, const bool dont_respond)
{
  if (hop >= regs.size())
    return;
  if (dont_respond)
    regs[hop].aux |= (uint32_t)2;
  else
    regs[hop].aux &= ~((uint32_t)2);
  OutboundPacket pkt;
  pkt.appendSetAux(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setTXInterval(const int hop, const uint16_t num_adc_ticks)
{
  if (hop >= regs.size())
    return;
  regs[hop].master &= ~((uint32_t)0xffff << 16);
  regs[hop].master |= ((uint32_t)num_adc_ticks) << 16;
  OutboundPacket pkt;
  pkt.appendSetMaster(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setUSBPower(const int hop, const uint8_t power_bits)
{
  if (hop >= regs.size())
    return;
  regs[hop].aux &= 0xffff00ff; // preserve everything but USB power bits
  regs[hop].aux |= ((uint32_t)power_bits) << 8;
  OutboundPacket pkt;
  pkt.appendSetAux(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setChainParameters(const int hop, const bool endpoint, 
                               const uint16_t payload_len)
{
  if (hop >= regs.size())
  {
    printf("ahh short chain\n");
    return;
  }
  regs[hop].net_endpoint = endpoint ? 1 : 0;
  regs[hop].net_payload_len = payload_len;
  OutboundPacket pkt;
  pkt.appendSetChainParameters(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setPWMParameters(const int hop)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendSetPWMParameters(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setEnableFETs(const int hop, const bool enable)
{
  if (hop >= regs.size() || hop >= mcs_.size())
    return;
  if (enable && mcs_[hop].enable_mosfets)
    regs[hop].master |= 1;
  else
    regs[hop].master &= ~((uint32_t)1);
  OutboundPacket pkt;
  pkt.appendSetMaster(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setPWMDuty(const int hop, const uint16_t a, const uint16_t b, const uint16_t c)
{
  if (hop >= regs.size())
    return;
  regs[hop].pwm_duty_a = a;
  regs[hop].pwm_duty_b = b;
  regs[hop].pwm_duty_c = c;
  OutboundPacket pkt;
  pkt.appendSetPWMDuty(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setFOCTarget(const int hop, const float amps)
{
  if (hop >= regs.size())
    return;
  regs[hop].foc_target = amps;
  OutboundPacket pkt;
  pkt.appendSetFOCTarget(hop, &regs[hop]);
  tx(pkt);
}
 
void Chain::setAddresses(const int hop)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendSetAddresses(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setFOCEnable(const int hop, const bool enable)
{
  if (hop >= regs.size())
    return;
  if (enable)
    regs[hop].master |= 0x04;
  else
    regs[hop].master &= ~0x04;
  OutboundPacket pkt;
  pkt.appendSetMaster(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setFOCParameters(const int hop)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendSetFOCParameters(hop, &regs[hop]);
  tx(pkt);
  // now send the "extra" block of FOC parameters for extended bonus features
  pkt.reset();
  pkt.appendSetExtraFOCParameters(hop, &regs[hop]);
  tx(pkt);
}

void Chain::setFootParameters(const int hop)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendSetFootParameters(hop, &regs[hop]);
  tx(pkt);
}

void Chain::txMCU(const int hop, const uint8_t *data, const uint8_t len)
{
  if (hop >= regs.size())
    return;
  OutboundPacket pkt;
  pkt.appendTxMCU(hop, data, len);
  tx(pkt);
}

void Chain::txMCUFrame(const int hop, const uint8_t *data, const uint8_t len)
{
  if (hop >= regs.size())
    return;
  uint8_t msg[256] = {0};
  memcpy(msg, data, len);
  uint8_t csum = 0;
  for (int i = 0; i < len; i++)
    csum += msg[i];
  msg[len] = csum;
  OutboundPacket pkt;
  pkt.appendTxMCU(hop, msg, len+1);
  tx(pkt);
}

