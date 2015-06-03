#ifndef WANDRR_LOCAL_MCAST_TX
#define WANDRR_LOCAL_MCAST_TX

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

namespace wandrr
{

class LocalMCastTx
{
public:
  LocalMCastTx(const char *mcast_group_cstr);
  ~LocalMCastTx() { } // todo

  int tx_sock_;
  sockaddr_in mcast_addr_;

  void tx(uint8_t *data, uint16_t data_len);
};

}

#endif

