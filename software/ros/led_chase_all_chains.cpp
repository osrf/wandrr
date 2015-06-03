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
  if (argc < 2)
  {
    printf("syntax: led_chase PARAMFILE\n");
    return 1;
  }
  if (!w.loadParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  int led_idx = 0, dir = 1;
  int num_nodes = 0;
  for (int i = 0; i < w.chains_.size(); i++)
    num_nodes += w.chains_[i]->regs.size();
  printf("%d nodes found on %d chains\n", num_nodes, (int)w.chains_.size());
  while (!g_done)
  {
    usleep(300000);
    if (dir)
    {
      led_idx++;
      if (led_idx == num_nodes - 1)
        dir = 0;
    }
    else
    {
      led_idx--;
      if (led_idx == 0)
        dir = 1;
    }
    printf("%d\n", led_idx);
    int nodes_visited = 0;
    for (int c = 0; c < (int)w.chains_.size(); c++)
    {
      OutboundPacket pkt;
      for (int i = 0; i < w.chains_[c]->regs.size(); i++)
      {
        bool on = false;
        if (nodes_visited + i == led_idx)
          on = true;
        if (on)
          printf("%d:%d ON\n", c, i);
        //bool on = led_idx == i;
        if (on)
          w.chains_[c]->regs[i].aux |= 1;
        else
          w.chains_[c]->regs[i].aux &= ~(uint32_t)1;
        pkt.appendSetAux(i, &w.chains_[c]->regs[i]);
      }
      w.tx(c, pkt);
      nodes_visited += (int)w.chains_[c]->regs.size();
    }
  }
  return 0;
}
