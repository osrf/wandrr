#ifndef SERIAL_ROUTER_H
#define SERIAL_ROUTER_H

#include <stdint.h>
#include <netinet/in.h>

namespace wandrr
{

#define NUM_DMXL 12
struct SerialRouterState
{
  uint32_t time_us;
  float    accels[3];
  float    gyros[3];
  float    mags[3];
  float    quaternion[4];
  uint16_t imu_sample_counter;
  float    dmxl_angle[NUM_DMXL];
  float    dmxl_vel[NUM_DMXL];
  float    dmxl_load[NUM_DMXL];
  float    dmxl_voltage[NUM_DMXL];
  float    dmxl_temp[NUM_DMXL];
  uint8_t  dmxl_status[NUM_DMXL];
} __attribute__((packed));

class SerialRouter
{
public:
  int tx_sock_, rx_sock_;
  sockaddr_in tx_addr_;

  SerialRouter(const char *iface);
  ~SerialRouter();

  bool setPortPower(const uint8_t port, const bool on);
  bool setDmxlLED(const uint8_t port, const uint8_t id, const bool on);
  bool setDmxlEnable(const uint8_t port, const uint8_t id, const bool on);
  bool setDmxlGoal(const uint8_t port, const uint8_t id, const uint16_t goal);
  bool setDmxlGoalRadians(const uint8_t port, const uint8_t id, const float goal_radians);
  bool setDmxlMaxTorque(const uint8_t port, const uint8_t id, const uint16_t max_torque);
  bool setWandrrDmxlGoals(const float *goals);
  void listen();
private:
  void tx(const uint8_t *data, const uint16_t len);
  SerialRouter();
  SerialRouter(const SerialRouter &);
};

}

#endif
