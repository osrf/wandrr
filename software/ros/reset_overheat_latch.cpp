#include "wandrr.h"
#include <cstdio>
using namespace wandrr;

int main(int argc, char **argv)
{
  if (argc < 4)
  {
    printf("syntax: reset_overheat_latch PARAMFILE CHAIN HOP\n");
    return 1;
  }
  Wandrr w;
  if (!w.loadParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }
  int c = atoi(argv[2]);
  int h = atoi(argv[3]);
  printf("resetting overheat latch for %d:%d ...\n", c, h);
  w.resetOverheatLatch(c, h);
  printf("done\n");
  return 0;
}
