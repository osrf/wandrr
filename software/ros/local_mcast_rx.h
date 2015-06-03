#ifndef WANDRR_LOCAL_MCAST_RX
#define WANDRR_LOCAL_MCAST_RX

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <boost/function.hpp>

namespace wandrr
{

class LocalMCastRx
{
public:
  LocalMCastRx(const char *mcast_group_cstr, const uint16_t port);
  ~LocalMCastRx() { } // todo

  int rx_sock_;
  sockaddr_in mcast_addr_;
  void listen(const double max_seconds);

  typedef boost::function<void(const uint8_t * const, const uint16_t)> MsgCallback;
  MsgCallback msg_cb_;

  void tx(uint8_t *data, uint16_t data_len);
  void setCallback(MsgCallback cb) { msg_cb_ = cb; }
};

}

#endif

