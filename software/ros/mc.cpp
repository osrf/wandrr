#include "mc.h"
using namespace wandrr;

MC::MC()
: enable_mosfets(false),
  has_foot(false),
  ignore_thermistor(false),
  inbound_data_valid(false),
  inbound_data_last_change_time(0),
  inbound_data_last_timestamp(0)
{
}

MC::~MC()
{
}
