#include "wandrr.h"
#include "outbound_packet.h"
#include <cstdio>
#include <signal.h>
using namespace wandrr;

bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}

int main(int argc, char **argv)
{
  Wandrr w;
  if (argc < 3)
  {
    printf("syntax: led_chase PARAMFILE NUM_NODES\n");
    return 1;
  }
  if (!w.loadParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }
  const int blink_length = atoi(argv[2]);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  int led_idx = 0, dir = 1;
  while (!g_done)
  {
    usleep(300000);
    if (dir)
    {
      led_idx++;
      if (led_idx == blink_length - 1)
        dir = 0;
    }
    else
    {
      led_idx--;
      if (led_idx == 0)
        dir = 1;
    }
    printf("%d\n", led_idx);
    OutboundPacket pkt;
    for (int i = 0; i < w.chains_[0]->regs.size(); i++)
    {
      bool on = led_idx == i;
      if (on)
        w.chains_[0]->regs[i].aux |= 1;
      else
        w.chains_[0]->regs[i].aux &= ~(uint32_t)1;
      pkt.appendSetAux(i, &w.chains_[0]->regs[i]);
    }
    w.tx(0, pkt);
  }
  return 0;
}
