#include <ros/ros.h>
#include "wandrr_controller.h"
#include "wandrr.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
#include <ifaddrs.h>
using namespace wandrr;

WandrrController::WandrrController(const char *iface)
{
  const char *mcast_addr_cstr = "224.0.0.123";
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
  /*
  int loopback = 0; // no loopback, thanks
  rc = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_LOOP,
                  (char *)&loopback, sizeof(loopback));
  Wandrr::perish_if(rc < 0, "couldn't turn off multicast loopback");
  */
  ////////////////////////////
  int reuseaddr = 1;
  rc = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
      &reuseaddr, sizeof(reuseaddr));
  Wandrr::perish_if(rc < 0, "couldn't set SO_REUSEADDR on UDP RX sock");

  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = mcast_addr_.sin_addr.s_addr; // INADDR_ANY;
  rx_bind_addr.sin_port = htons(11303);

  rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port 11303");

  printf("binding sock %d to %s\n", rx_sock_, mcast_addr_cstr);

  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_cstr);
  mreq.imr_interface.s_addr = local_ipv4_addr;
  rc = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  &mreq, sizeof(mreq));
  Wandrr::perish_if(rc < 0, "couldn't add ourselves to the multicast group");
}

WandrrController::~WandrrController()
{
}

void WandrrController::listen(const double max_seconds)
{
  ros::Time t_start(ros::Time::now());
  static uint8_t buf[1500] = {0};
  while (ros::ok())
  {
    timeval timeout;
    timeout.tv_sec = (time_t)trunc(max_seconds);
    timeout.tv_usec = (suseconds_t)((max_seconds - timeout.tv_sec) * 1e6);
    fd_set rdset;
    FD_ZERO(&rdset);
    FD_SET(rx_sock_, &rdset);
    int rv = select(rx_sock_ + 1, &rdset, NULL, NULL, &timeout);
    if (rv < 0)
    {
      ROS_ERROR("error in select()");
      return;
    }
    if (rv > 0)
    {
      const int nbytes = recvfrom(rx_sock_, buf, sizeof(buf), 0, NULL, NULL);
      if (nbytes < 0)
      {
        ROS_ERROR("error in recvfrom");
        return;
      }
      if (nbytes == 1048)
      {
        if (state_cb_)
          state_cb_(this, (const StepprMegaState * const)buf);
      }
      else
        ROS_WARN("rx %d bytes from recvfrom. expected 1048", nbytes);
    }
    ros::Time t(ros::Time::now());
    if ((t - t_start).toSec() > max_seconds)
      break; // all done. time's up.
  }
}

void WandrrController::txMegaCmd(StepprMegaCmd *mc)
{
  mcast_addr_.sin_port = htons(11305);
  int nsent = sendto(tx_sock_, mc, sizeof(StepprMegaCmd), 0,
                     (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
}

