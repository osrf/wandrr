#ifndef NODE_H
#define NODE_H

#include <stdint.h>

// todo: pull regs into this object, to model the system more cleanly 
namespace wandrr
{

class MC
{
public:
  MC();
  ~MC();

  bool enable_mosfets;
  bool has_foot;
  bool ignore_thermistor;
  bool inbound_data_valid;
  double inbound_data_last_change_time;
  uint64_t inbound_data_last_timestamp;
};

}

#endif
