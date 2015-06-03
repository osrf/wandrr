#include "local_mcast_tx.h"
#include <ros/console.h>
#include <unistd.h>
#include <arpa/inet.h>
using namespace wandrr;

LocalMCastTx::LocalMCastTx(const char *mcast_group_cstr)
{
  tx_sock_     = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_FATAL_COND(tx_sock_     < 0, "SI: couldn't create socket");
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_group_cstr);
  in_addr local_addr;
  local_addr.s_addr = inet_addr("127.0.0.1");
  int result = 0;
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF,
                      (char *)&local_addr, sizeof(local_addr));
  ROS_FATAL_COND(result < 0, "couldn't set local interface for udp tx sock");
}

void LocalMCastTx::tx(uint8_t *data, uint16_t data_len)
{
  mcast_addr_.sin_port = htons(11399);
  int nsent = sendto(tx_sock_, data, data_len, 0,
                     (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
}
