#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "power_router.h"
using namespace wandrr;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: power_router_cli IFACE COMMAND [OPTIONS]\n");
    return 1;
  }
  const char *iface = argv[1];
  const char *cmd = argv[2];
  PowerRouter sr(iface);
  if (!strcmp(cmd, "power"))
  {
    const int port = atoi(argv[3]);
    const int on = atoi(argv[4]);
    sr.setPortPower(port, on);
  }
  else if (!strcmp(cmd, "listen"))
  {
    sr.listen();
  }
  return 0;
}
