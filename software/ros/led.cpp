#include "wandrr.h"
#include <cstdio>
using namespace wandrr;

int main(int argc, char **argv)
{
  if (argc < 5)
  {
    printf("syntax: led PARAMFILE CHAIN HOP {1 | 0}\n");
    return 1;
  }
  Wandrr w;
  if (!w.loadParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }
  printf("setting LED...\n");
  w.setLED(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]) ? true : false);
  printf("done\n");
  return 0;
}
