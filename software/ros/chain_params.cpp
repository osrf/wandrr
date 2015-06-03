#include "wandrr.h"
#include <cstdio>
using namespace wandrr;

int main(int argc, char **argv)
{
  if (argc < 6)
  {
    printf("syntax: chain_params PARAMFILE CHAIN NODE ENDPOINT PAYLOAD_LEN\n");
    return 1;
  }
  Wandrr w;
  if (!w.initFromParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }
  usleep(100000);
  w.setChainParameters(atoi(argv[2]), 
                       atoi(argv[3]), 
                       atoi(argv[4]) ? true : false, 
                       atoi(argv[5]));
  usleep(100000);
  return 0;
}
