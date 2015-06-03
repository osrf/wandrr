#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "serial_router.h"
using namespace wandrr;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: serial_router_cli INTERFACE COMMAND [OPTIONS]\n");
    return 1;
  }
  SerialRouter sr(argv[1]);
  const char *cmd = argv[2];
  if (!strcmp(cmd, "power"))
  {
    const int port = atoi(argv[3]);
    const int on = atoi(argv[4]);
    printf("setting port %d %s\n", port, on ? "on" : "off");
    sr.setPortPower(port, on);
  }
  else if (!strcmp(cmd, "dmxl_led"))
  {
    const int port = atoi(argv[3]);
    const int id = atoi(argv[4]);
    const int on = atoi(argv[5]);
    sr.setDmxlLED(port, id, on);
  }
  else if (!strcmp(cmd, "dmxl_enable"))
  {
    const int port = atoi(argv[3]);
    const int id = atoi(argv[4]);
    const int en = atoi(argv[5]);
    sr.setDmxlEnable(port, id, en);
  }
  else if (!strcmp(cmd, "dmxl_goal"))
  {
    const int port = atoi(argv[3]);
    const int id = atoi(argv[4]);
    const int goal = atoi(argv[5]);
    sr.setDmxlGoal(port, id, goal);
  }
  else if (!strcmp(cmd, "dmxl_max_torque"))
  {
    const int port = atoi(argv[3]);
    const int id = atoi(argv[4]);
    const int max_torque = atoi(argv[5]);
    sr.setDmxlMaxTorque(port, id, max_torque);
  }
  else if (!strcmp(cmd, "listen"))
  {
    sr.listen();
  }
  return 0;
}
