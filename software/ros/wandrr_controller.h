#ifndef WANDRR_CONTROLLER_H
#define WANDRR_CONTROLLER_H

#include "steppr_interface.h"
#include <boost/function.hpp>
#include <netinet/in.h>

namespace wandrr
{

class WandrrController
{
public:
  typedef boost::function<void(WandrrController *wc, const StepprMegaState * const)> StateCallback;

  WandrrController(const char *iface);
  ~WandrrController();

  int tx_sock_, rx_sock_;
  StateCallback state_cb_;
  sockaddr_in mcast_addr_;

  void listen(const double max_seconds);

  void setStateCallback(StateCallback cb) { state_cb_ = cb; }
  void txMegaCmd(StepprMegaCmd *mc);
};

}

#endif

