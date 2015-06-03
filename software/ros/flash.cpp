#include "wandrr.h"
#include <cstdio>
#include <ros/ros.h>
using namespace wandrr;

bool g_txrx_done = false;
bool g_chain_txrx_done[8] = {false};
uint8_t g_rxbuf[2048] = {0};
int g_rxlen = 0;
static const uint8_t xfer_size = 32;

static int g_chain_tgt = -1;
static int g_node_tgt = -1;

void mcu_rx_cb(int chain, int hop, const uint8_t *data, const uint16_t len)
{
  //printf("mcu rx %d bytes\n", len);
  /*
  for (int i = 0; i < len; i++)
    printf("%02x ", data[i]);
  printf("\n");
  */
  if (chain != g_chain_tgt ||
      hop != g_node_tgt)
    return; // adios
  if (len > sizeof(g_rxbuf))
  {
    printf("woah. huge rxlen: %d\n", len);
    return;
  }
  memcpy(g_rxbuf, data, len);
  g_rxlen = len;
  g_txrx_done = true;
  if (hop < 8)
    g_chain_txrx_done[hop] = true;
}

bool erase_sector(Wandrr *w, const int chain, const int hop, const uint32_t start_addr)
{
  printf("erasing sector 0x%08x\n", (unsigned)start_addr);
  uint8_t mcu_pkt[xfer_size+8] = {0};
  mcu_pkt[0] = 1; // flash command
  mcu_pkt[1] = 2; // subcommand: erase
  memcpy(&mcu_pkt[4], &start_addr, 4);
  for (int attempt = 0; attempt < 3; attempt++)
  {
    g_txrx_done = false;
    w->txMCUFrame(chain, hop, mcu_pkt, 8);
    ros::Time t_start(ros::Time::now());
    while ((ros::Time::now() - t_start).toSec() < 4.0)
    {
      w->listen(0.00001);
      if (g_txrx_done)
        break;
    }
    if (!g_txrx_done)
    {
      printf("no response to erase command\n");
      continue;
    }
    break;
  }
  return g_txrx_done;
}

bool configure(Wandrr *w, const int chain, const int hop)
{
  printf("starting configuration...\n");
  uint8_t mcu_pkt[8] = {0};
  mcu_pkt[0] = 2; // command: fpga
  mcu_pkt[1] = 1; // subcommand: configure
  w->txMCUFrame(chain, hop, mcu_pkt, 8);
  return true;
}

bool dump_micropage(Wandrr *w, const int chain, const int hop,
                    const int start_addr,
                    uint8_t *rx_data)
{
  uint8_t mcu_pkt[64] = {0}; 
  mcu_pkt[0] = 1; // flash command
  mcu_pkt[1] = 0; // subcommand: read
  mcu_pkt[2] = 32; // requested transaction length = 32
  for (int attempt = 0; attempt < 10; attempt++)
  {
    memcpy(&mcu_pkt[4], &start_addr, 4);
    g_txrx_done = false;
    w->txMCUFrame(chain, hop, mcu_pkt, 8);
    ros::Time t_start(ros::Time::now());
    while (ros::ok() && (ros::Time::now() - t_start).toSec() < 0.2)
    {
      w->listen(0.001);
      if (g_txrx_done)
      {
        if (rx_data)
          memcpy(rx_data, g_rxbuf, 32);
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wandrr_flash");
  ros::NodeHandle nh;
  Wandrr w;
  if (argc < 5)
  {
    printf("syntax: flash PARAMFILE CHAIN NODE [COMMAND [OPTIONS]]\n");
    printf("            COMMAND = { dump, burn, configure }\n");
    return 1;
  }
  if (!w.initFromParamFile(argv[1]))
  {
    printf("couldn't load motor-chain configuration file\n");
    return 1;
  }
  const int chain = atoi(argv[2]);
  const int hop = atoi(argv[3]);
  g_chain_tgt = chain;
  g_node_tgt = hop;
  printf("talking to %d:%d\n", chain, hop);
  const char *cmd = argv[4];
  w.setMCURXCallback(mcu_rx_cb);
  for (int c = 0; c < (int)w.chains_.size(); c++)
  {
    int max_hop = w.chains_[c]->regs.size()-1;
    for (int i = max_hop; i >= 0; i--)
    {
      w.setTXInterval(c, i, 65535);
      w.setChainParameters(c, i, false, 800); // nodes upstream need to passthru
    }
  }
  w.setChainParameters(chain, hop, true, 800);
  usleep(100000);
  uint8_t mcu_pkt[xfer_size+8] = {0};
  mcu_pkt[0] = 1; // flash command
  mcu_pkt[2] = xfer_size; // requested transaction length
  if (!strcmp(cmd, "dump"))
  {
    mcu_pkt[1] = 0; // subcommand: read
    FILE *f_dump = fopen("dump", "wb");
    printf("dumping flash...\n");
    for (uint32_t read_addr = 0; read_addr < 2632640; read_addr += xfer_size)
    {
      for (int attempt = 0; ros::ok() && attempt < 10; attempt++)
      {
        if (read_addr % 0x1000 == 0)
          printf("reading 0x%08x (%d) = %.1f %%\n", 
                 (unsigned)read_addr, read_addr, 
                 100.0f * read_addr / 2632640.0f);
        memcpy(&mcu_pkt[4], &read_addr, 4);
        g_txrx_done = false;
        w.txMCUFrame(chain, hop, mcu_pkt, 8);
        ros::Time t_start(ros::Time::now());
        while (ros::ok() && (ros::Time::now() - t_start).toSec() < 2.0)
        {
          w.listen(0.000001);
          if (g_txrx_done)
            break;
        }
        if (!g_txrx_done)
        {
          printf("no response to command\n");
          continue;
        }
        if (g_rxbuf[2] != xfer_size)
        {
          printf("unexpected rx len: %d\n", g_rxbuf[2]);
          continue;
        }
        fwrite(&g_rxbuf[8], 1, xfer_size, f_dump);
        break;
      }
      if (!g_txrx_done)
      {
        printf("stopping.\n");
        break;
      }
    }
    fclose(f_dump);
  }
  else if (!strcmp(cmd, "burn"))
  {
    if (argc < 6)
    {
      printf("usage: flash PARAMFILE CHAIN NODE burn FILENAME\n");
      return 1;
    }
    mcu_pkt[1] = 1; // subcommand: write
    const char *filename = argv[5];
    FILE *f_burn = fopen(filename, "rb");
    if (!f_burn)
    {
      printf("couldn't open [%s]\n", filename);
      return 1;
    }
    if (!dump_micropage(&w, chain, hop, 0, NULL))
    {
      printf("couldn't establish comms with target board.\n");
      printf("let's stop now before we start erasing things.\n");
      return 1;
    }
    uint32_t write_addr = 0;
    while (!feof(f_burn))
    {
      if ((write_addr & 0xffff) == 0)
        if (!erase_sector(&w, chain, hop, write_addr))
          break;
      if (write_addr % 0x1000 == 0)
        printf("writing 0x%08x (%d) = %0.1f %%\n", 
               (unsigned)write_addr, write_addr,
               100.0f * write_addr / 2632640.0f);
      memcpy(&mcu_pkt[4], &write_addr, 4);
      memset(&mcu_pkt[8], 0, 32);
      size_t nread = fread(&mcu_pkt[8], 1, 32, f_burn);
      if (nread != 32)
        printf("read %d bytes. last page?\n", (int)nread);
      if (nread == 0)
        break;
      for (int attempt = 0; attempt < 10; attempt++)
      {
        g_txrx_done = false;
        w.txMCUFrame(chain, hop, mcu_pkt, 8 + 32);
        ros::Time t_start(ros::Time::now());
        while (ros::ok() && (ros::Time::now() - t_start).toSec() < 0.1)
        {
          w.listen(0.00001);
          if (g_txrx_done)
            break;
        }
        if (!g_txrx_done)
        {
          printf("no response to command\n");
          continue;
        }
        // todo: check error code?
        break;
      }
      if (!g_txrx_done)
      {
        printf("stopping.\n");
        break;
      }
      write_addr += 32;
    }
  }
  else if (!strcmp(cmd, "burn_chain"))
  {
    if (argc < 5)
    {
      printf("usage: flash PARAMFILE CHAIN NODE_MAX burn_chain FILENAME\n");
      return 1;
    }
    mcu_pkt[1] = 1; // subcommand: write
    const char *filename = argv[5];
    FILE *f_burn = fopen(filename, "rb");
    if (!f_burn)
    {
      printf("couldn't open [%s]\n", filename);
      return 1;
    }
    uint32_t write_addr = 0;
    while (!feof(f_burn))
    {
      if ((write_addr & 0xffff) == 0)
      {
        printf("erasing sector 0x%08x\n", (unsigned)write_addr);
        uint8_t mcu_pkt[xfer_size+8] = {0};
        mcu_pkt[0] = 1; // flash command
        mcu_pkt[1] = 2; // subcommand: erase
        memcpy(&mcu_pkt[4], &write_addr, 4);
        for (int h = 0; h <= hop; h++)
          g_chain_txrx_done[h] = false;
        for (int attempt = 0; attempt < 3; attempt++)
        {
          for (int h = 0; h <= hop; h++)
          {
            if (g_chain_txrx_done[h])
              continue;
            w.txMCUFrame(chain, h, mcu_pkt, 8);
          }
          ros::Time t_start(ros::Time::now());
          while ((ros::Time::now() - t_start).toSec() < 4.0)
          {
            w.listen(0.00001);
            bool all_done = true;
            for (int h = 0; h <= hop; h++)
              if (!g_chain_txrx_done[h])
                all_done = false;
            if (all_done)
              break;
          }
          for (int h = 0; h <= hop; h++)
            if (!g_chain_txrx_done[h])
            {
              printf("no response to erase command from node %d\n", h);
              continue;
            }
          bool all_done = true;
          for (int h = 0; h <= hop; h++)
            if (!g_chain_txrx_done[h])
              all_done = false;
          if (all_done)
            break; // if we get here, that means that all nodes up to NODE_MAX are OK
          printf("  attempt %d/10 erasing page 0x%08x...\n", attempt+1, (unsigned)write_addr);
        }
        for (int h = 0; h <= hop; h++)
          if (!g_chain_txrx_done[h])
            break;
      }
      mcu_pkt[1] = 1; // subcommand: write
      if (write_addr % 0x1000 == 0)
        printf("writing 0x%08x (%d) = %0.1f %%\n", 
               (unsigned)write_addr, write_addr,
               100.0f * write_addr / 2632640.0f);
      memcpy(&mcu_pkt[4], &write_addr, 4);
      memset(&mcu_pkt[8], 0, 32);
      size_t nread = fread(&mcu_pkt[8], 1, 32, f_burn);
      if (nread != 32)
        printf("read %d bytes. last page?\n", (int)nread);
      if (nread == 0)
        break;
      for (int h = 0; h <= hop; h++)
        g_chain_txrx_done[h] = false;
      for (int attempt = 0; attempt < 10; attempt++)
      {
        for (int h = 0; h <= hop; h++)
        {
          if (g_chain_txrx_done[h])
            continue;
          w.txMCUFrame(chain, h, mcu_pkt, 8 + 32);
        }
        ros::Time t_start(ros::Time::now());
        while (ros::ok() && (ros::Time::now() - t_start).toSec() < 0.1)
        {
          w.listen(0.00001);
          bool all_done = true;
          for (int h = 0; h <= hop; h++)
            if (!g_chain_txrx_done[h])
              all_done = false;
          if (all_done)
            break;
        }
        if (!g_txrx_done)
        {
          printf("no response to command\n");
          continue;
        }
        bool all_done = true;
        for (int h = 0; h <= hop; h++)
          if (!g_chain_txrx_done[h])
            all_done = false;
        if (all_done)
          break; // if we get here, that means that all nodes up to NODE_MAX are OK
        printf("  attempt %d/10 writing page 0x%08x...\n", attempt+1, (unsigned)write_addr);
      }
      bool abort_requested = false;
      for (int h = 0; h <= hop; h++)
        if (!g_chain_txrx_done[h])
        {
          printf("abort.\n");
          abort_requested = true;
          break;
        }
      if (abort_requested)
        break;
      write_addr += 32;
    }

  }
  else if (!strcmp(cmd, "configure"))
    configure(&w, chain, hop);
  else
  {
    printf("unknown command: [%s]\n", cmd);
    return 1;
  }
  return 0;
}

