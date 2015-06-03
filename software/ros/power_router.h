#ifndef POWER_ROUTER_H
#define POWER_ROUTER_H

#include <stdint.h>
#include <netinet/in.h>

namespace wandrr
{

struct PowerRouterState
{
  uint32_t time_us;
  uint8_t channels_active;
  uint8_t unused[3]; // for alignment
  int16_t adc[8];
  float battery_voltage[2];
  float battery_current[2];
  float battery_celsius[2];
} __attribute((packed));

class PowerRouter
{
public:
  int tx_sock_, rx_sock_;
  sockaddr_in tx_addr_;

  PowerRouter(const char *iface);
  ~PowerRouter();

  bool setPortPower(const uint8_t port, const bool on);
  void listen();
private:
  void tx(const uint8_t *data, const uint16_t len);
  PowerRouter();
  PowerRouter(const PowerRouter &);
};

}

#endif
