#include "wandrr.h"
#include <cstdio>
using namespace wandrr;

int main(int argc, char **argv)
{
  Wandrr w;
  if (argc < 3)
  {
    printf("syntax: tx_mcu NODE BYTES\n");
    return 1;
  }
  uint8_t hop = atoi(argv[1]);
  uint8_t len = atoi(argv[2]);
  uint8_t dummy_payload[128];
  for (int i = 0; i < len; i++)
    dummy_payload[i] = i+1;
  while (1)
  {
    usleep(100000);
    w.txMCU(hop, dummy_payload, len);
  }
  return 0;
}
