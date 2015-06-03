#include "local_mcast_rx.h"
#include "wandrr.h"
#include <ros/ros.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
using namespace wandrr;

LocalMCastRx::LocalMCastRx(const char *mcast_group_cstr, const uint16_t port)
{
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  Wandrr::perish_if(rx_sock_ < 0, "couldn't create rx socket");
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_group_cstr);
  int reuseaddr = 1;
  int rc = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
                      &reuseaddr, sizeof(reuseaddr));
  Wandrr::perish_if(rc < 0, "couldn't set SO_REUSEADDR on UDP RX sock");
  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = INADDR_ANY;
  rx_bind_addr.sin_port = htons(port);
  rc = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  Wandrr::perish_if(rc < 0, "couldn't bind rx sock to port");
  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_group_cstr);
  mreq.imr_interface.s_addr = inet_addr("127.0.0.1");
  rc = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                  &mreq, sizeof(mreq));
  Wandrr::perish_if(rc < 0, "couldn't add ourselves to the multicast group");
}

void LocalMCastRx::listen(const double max_seconds)
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
      if (msg_cb_)
        msg_cb_(buf, nbytes);
    }
    ros::Time t(ros::Time::now());
    if ((t - t_start).toSec() > max_seconds)
      break; // all done. time's up.
  }
}
