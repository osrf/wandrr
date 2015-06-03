#include "serial_router.h"
#include "wandrr.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <cstdio>
#include "../../../firmware/wandrr/sr/common/state.h"
using namespace wandrr;

SerialRouter::SerialRouter(const char *iface)
{
  //printf("serial router starting initialization on iface %s\n", iface);
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  Wandrr::perish_if(tx_sock_ < 0, "couldn't create tx socket");
  Wandrr::perish_if(rx_sock_ < 0, "couldn't create rx socket");
  tx_addr_.sin_family = AF_INET;
  tx_addr_.sin_addr.s_addr = inet_addr("10.66.176.89");
  tx_addr_.sin_port = htons(11333);
  /*
  int reuseaddr = 1;
  int rc = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
                      &reuseaddr, sizeof(reuseaddr));
  Wandrr::perish_if(rc < 0, "couldn't set SO_REUSEADDR on UDP RX sock");
  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  rx_bind_addr.sin_port = htons(11333);
  */
  //rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  //Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port 11333");

  //mcast_addr_.sin_family = AF_INET;
  //mcast_addr_.sin_addr.s_addr = inet_addr(mcast_addr_cstr);
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
  ////////////////////////////
  int reuseaddr = 1;
  rc = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
      &reuseaddr, sizeof(reuseaddr));
  Wandrr::perish_if(rc < 0, "couldn't set SO_REUSEADDR on UDP RX sock");

  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  const char *mcast_addr_cstr = "224.0.0.141";
  rx_bind_addr.sin_addr.s_addr = inet_addr(mcast_addr_cstr); //(mcast_addr_.sin_addr.s_addr; // INADDR_ANY;
  rx_bind_addr.sin_port = htons(11333);

  rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port");

  //printf("binding sock %d to %s\n", rx_sock_, mcast_addr_cstr);

  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_cstr);
  mreq.imr_interface.s_addr = local_ipv4_addr;
  rc = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  &mreq, sizeof(mreq));
  Wandrr::perish_if(rc < 0, "couldn't add ourselves to the multicast group");
}

SerialRouter::~SerialRouter()
{
  // todo
}

void SerialRouter::tx(const uint8_t *data, const uint16_t len)
{
  int nsent = sendto(tx_sock_, data, len, 0,
                     (sockaddr *)&tx_addr_, sizeof(tx_addr_));
  if (nsent != len)
    printf("woah, sendto() returned %d\n", nsent);
}

bool SerialRouter::setPortPower(const uint8_t port, const bool on)
{
  uint8_t pkt[16];
  pkt[0] = 1; // packet type 1 = set port power
  pkt[1] = port;
  pkt[2] = on ? 1 : 0;
  tx(pkt, 3);
}

bool SerialRouter::setDmxlLED(const uint8_t port, const uint8_t id, 
                              const bool on)
{
  uint8_t pkt[16];
  pkt[0] = 2; // packet type 2 = write dynamixel data
  pkt[1] = port;
  pkt[2] = id;
  pkt[3] = 25; // dynamixel address 25 = LED
  pkt[4] = 1; // write only one byte
  pkt[5] = on ? 1 : 0;
  tx(pkt, 6);
}

bool SerialRouter::setDmxlEnable(const uint8_t port, const uint8_t id, 
                                 const bool on)
{
  uint8_t pkt[16];
  pkt[0] = 2; // packet type 2 = write dynamixel data
  pkt[1] = port;
  pkt[2] = id;
  pkt[3] = 24; // dynamixel address 24 = motor enable
  pkt[4] = 1; // write only one byte
  pkt[5] = on ? 1 : 0;
  tx(pkt, 6);
}

bool SerialRouter::setDmxlGoal(const uint8_t port, const uint8_t id, 
                               const uint16_t goal)
{
  uint8_t pkt[16];
  pkt[0] = 2; // packet type 2 = write dynamixel data
  pkt[1] = port;
  pkt[2] = id;
  pkt[3] = 30; // dynamixel address 30/31 = goal position
  pkt[4] = 2; // write two bytes
  pkt[5] = goal & 0xff;
  pkt[6] = goal >> 8;
  tx(pkt, 7);
}

bool SerialRouter::setDmxlMaxTorque(const uint8_t port, const uint8_t id, const uint16_t max_torque)
{
  printf("setting max torque on %d:%d to %d\n",
         port, id, max_torque);
  uint8_t pkt[16];
  pkt[0] = 2; // packet type 2 = write dynamixel data
  pkt[1] = port;
  pkt[2] = id;
  pkt[3] = 14; // dynamixel address 14/15 = max torque (1023 = max)
  pkt[4] = 2; // write two bytes
  pkt[5] = max_torque & 0xff;
  pkt[6] = max_torque >> 8;
  tx(pkt, 7);
}

static void printStateChain(state_t *s, const char *name, int c, int start, int len)
{
  printf("%s:\n", name);
  printf("  angles = [ ");
  for (int i = 0; i < len; i++)
    printf("%8.3f ", s->dmxl_angle[start+i]);
  printf("]\n  status = [ ");
  for (int i = 0; i < len; i++)
    printf("   0x%02x  ", s->dmxl_status[start+i]);
  printf("]\n  temps  = [ ");
  for (int i = 0; i < len; i++)
    printf("%5.0f    ", s->dmxl_temp[start+i]);
  printf("]\n");
}

static void printState(state_t *s)
{
  printf("\n\n\nt = %d\n"
         "accels = [%.3f %.3f %.3f]\n"
         "quat = [%.3f %.3f %.3f %.3f]\n",
         s->time_us,
         s->accels[0], s->accels[1], s->accels[2],
         s->quaternion[0], s->quaternion[1],
         s->quaternion[2], s->quaternion[3]);
  printStateChain(s, "left arm", 0, 0, 5);
  printStateChain(s, "right arm", 1, 5, 5);
  printStateChain(s, "neck", 2, 10, 2);
}

void SerialRouter::listen()
{
  static uint8_t buf[1500] = {0};
  while (true)
  {
    sockaddr_in rx_addr;
    socklen_t rx_addr_len = sizeof(rx_addr);
    const int nbytes = recvfrom(rx_sock_, buf, sizeof(buf), 0, 
        (sockaddr *)&rx_addr, &rx_addr_len);
    if (nbytes < 0)
    {
      printf("error in recvfrom\n");
      break;
    }
    else if (nbytes == sizeof(state_t))
    {
      state_t *s = (state_t *)buf;
      static int rx_count = 0;
      if (rx_count++ % 20 == 0) // print at 50 hz
      {
        printState(s);
      }
    }
    else
      printf("unexpected rx %d bytes\n", nbytes);
  }
}

bool SerialRouter::setDmxlGoalRadians(const uint8_t port, const uint8_t id, const float goal_radians)
{
  return setDmxlGoal(port, id, goal_radians * 2047.0 / 3.14159 + 2048.0);
}

bool SerialRouter::setWandrrDmxlGoals(const float *goals_radians)
{
  // convert float (radian) goals to dmxl scale
  uint16_t goals_ticks[12];
  for (int i = 0; i < 12; i++)
  {
    goals_ticks[i] = goals_radians[i] * 2047.0 / 3.14159 + 2048.0;
    //if (i == 0)
    //  printf("  ticks = %d\n", goals_ticks[i]);
    //printf(" %6d ", goals_ticks[i]);
    //printf("%d ", goals_ticks[i]);
  }
  //printf("\n");
  uint8_t pkt[25] = {0};
  pkt[0] = 3; // packet type 3 = wandrr-specific goal vector
  memcpy(pkt+1, goals_ticks, 24);
  tx(pkt, 25);
}

