#include "power_router.h"
#include "wandrr.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <cstdio>
#include "../../firmware/power_router/common/state.h"
using namespace wandrr;

PowerRouter::PowerRouter(const char *iface)
{
  printf("power router starting initialization on iface %s\n", iface);
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  Wandrr::perish_if(tx_sock_ < 0, "couldn't create tx socket");
  Wandrr::perish_if(rx_sock_ < 0, "couldn't create rx socket");
  tx_addr_.sin_family = AF_INET;
  tx_addr_.sin_addr.s_addr = inet_addr("10.66.177.89");
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
  const char *mcast_addr_cstr = "224.0.0.142";
  rx_bind_addr.sin_addr.s_addr = inet_addr(mcast_addr_cstr); //(mcast_addr_.sin_addr.s_addr; // INADDR_ANY;
  rx_bind_addr.sin_port = htons(11333);

  rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port");

  printf("binding sock %d to %s\n", rx_sock_, mcast_addr_cstr);

  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_cstr);
  mreq.imr_interface.s_addr = local_ipv4_addr;
  rc = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  &mreq, sizeof(mreq));
  Wandrr::perish_if(rc < 0, "couldn't add ourselves to the multicast group");
}

PowerRouter::~PowerRouter()
{
  // todo
}

void PowerRouter::tx(const uint8_t *data, const uint16_t len)
{
  int nsent = sendto(tx_sock_, data, len, 0,
                     (sockaddr *)&tx_addr_, sizeof(tx_addr_));
  if (nsent != len)
    printf("woah, sendto() returned %d\n", nsent);
}

bool PowerRouter::setPortPower(const uint8_t port, const bool on)
{
  uint8_t pkt[16];
  pkt[0] = 1; // packet type 1 = set port power
  pkt[1] = port;
  pkt[2] = on ? 1 : 0;
  tx(pkt, 3);
}

void PowerRouter::listen()
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
      printf("t = %d ch_states = 0x%x adc = [%7d %7d %7d %7d %7d %7d %7d %7d]\n",
              s->time_us,
              (unsigned)s->channels_active,
              s->adc[0], s->adc[1], s->adc[2], s->adc[3],
              s->adc[4], s->adc[5], s->adc[6], s->adc[7]);
      printf(" voltage = [ %.3f %.3f ]\n", 
             s->battery_voltage[0], s->battery_voltage[1]);
      printf(" current = [ %.3f %.3f ]\n", 
             s->battery_current[0], s->battery_current[1]);
      printf(" celsius = [ %.1f %.1f ]\n",
             s->battery_celsius[0], s->battery_celsius[1]);
    }
    else
      printf("unexpected rx %d bytes\n", nbytes);
  }
}
