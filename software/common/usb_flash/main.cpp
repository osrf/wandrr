#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <errno.h>

#define VENDOR_ID  0xf055
#define PRODUCT_ID 0x0123

void perish_if(bool b, const char *msg)
{
  if (b)
  {
    printf("%s\n", msg);
    exit(1);
  }
}

static bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

int usage()
{
  puts("usage: usb_flash COMMAND [OPTIONS]\n");
  puts("  where COMMAND is one of { dump, burn FILENAME }\n");
  return 1;
}

void dump(libusb_device_handle *h)
{
  FILE *f = fopen("dump.bin", "wb");
  for (uint32_t addr = 0; addr < 2632640; addr += 32)
  {
    uint8_t tx_msg[64] = {0};
    tx_msg[0] = 1;  // flash command
    tx_msg[1] = 0;  // read memory
    tx_msg[2] = 32; // read length
    int num_transferred = 0;
    memcpy(&tx_msg[4], &addr, sizeof(addr));
    int rc = libusb_bulk_transfer(h, 2, tx_msg, sizeof(tx_msg), 
                                  &num_transferred, 100);
    if (rc != 0)
    {
      printf("tx rc = %d\n", rc);
      break;
    }
    uint8_t rx_msg[64] = {0};
    int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                     rx_msg, sizeof(rx_msg),
                                     &num_transferred, 50);
    if (rx_rc != 0)
    {
      printf("rx err code: %d\n", rx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }

    if (num_transferred != 64)
    {
      printf("didn't receive a full packet in response\n");
      break;
    }
    fwrite(&rx_msg[8], 1, 32, f);
  }
  fclose(f);
}

bool erase_sector(libusb_device_handle *h, uint32_t start_addr)
{
  uint8_t msg[64] = {0};
  msg[0] = 1; // command: flash
  msg[1] = 2; // subcmd: erase sector
  memcpy(&msg[4], &start_addr, sizeof(start_addr));
  int usb_num_transferred = 0;
  int tx_rc = libusb_bulk_transfer(h, 2, msg, sizeof(msg),
                                   &usb_num_transferred, 100);
  if (tx_rc != 0)
  {
    printf("tx_rc = %d\n", tx_rc);
    return false;
  }
  uint8_t rx_msg[64] = {0};
  int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                   rx_msg, sizeof(rx_msg),
                                   &usb_num_transferred, 3000); // 3s timeout
  if (rx_rc != 0)
  {
    printf("erase cmd rx err code: %d\n", rx_rc);
    printf("errno: %d = %s\n", errno, strerror(errno));
    return false;
  }
  uint32_t response_write_addr = 0;
  memcpy(&response_write_addr, &rx_msg[4], sizeof(uint32_t));
  if (start_addr != response_write_addr)
  {
    printf("erase addr mismatch: 0x%08x != 0x%08x\n",
           response_write_addr, start_addr);
    return false;
  }
  return true;
}

void burn(libusb_device_handle *h, const char *filename)
{
  printf("burning %s\n", filename);
  FILE *f = fopen(filename, "rb");
  if (!f)
  {
    printf("couldn't open %s\n", filename);
    exit(1);
  }
  uint32_t write_addr = 0; // this image will start to be burned at zero
  uint8_t tx_msg[64] = {0};
  tx_msg[0] =  1; // command: flash
  tx_msg[1] =  1; // subcmd: write
  tx_msg[2] = 32; // write length
  int num_transferred = 0;
  while (!feof(f))
  {
    if ((write_addr & 0xffff) == 0)
    {
      // we're on a new sector boundary. erase it so we can program it.
      printf("erasing sector starting at 0x%08x\n", (unsigned)write_addr);
      if (!erase_sector(h, write_addr))
        break;
    }
    if (write_addr % 0x1000 == 0)
      printf("writing to 0x%08x\r\n", write_addr);
    //if (write_addr >= 0x10000)
    //  break;
    memcpy(&tx_msg[4], &write_addr, sizeof(write_addr));
    memset(&tx_msg[8], 0, 32);
    size_t nread = fread(&tx_msg[8], 1, 32, f);
    if (nread != 32)
      printf("read %d bytes. last page?\n", (int)nread);
    if (nread == 0)
      break;
    int tx_rc = libusb_bulk_transfer(h, 2, tx_msg, sizeof(tx_msg),
                                     &num_transferred, 100);
    if (tx_rc != 0)
    {
      printf("tx_rc = %d\n", tx_rc);
      break;
    }
    uint8_t rx_msg[64] = {0};
    int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                     rx_msg, sizeof(rx_msg),
                                     &num_transferred, 100);
    if (rx_rc != 0)
    {
      printf("rx err code: %d\n", rx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }
    uint32_t response_write_addr = 0;
    memcpy(&response_write_addr, &rx_msg[4], sizeof(uint32_t));
    if (write_addr != response_write_addr)
    {
      printf("write addr mismatch: 0x%08x != 0x%08x\n",
             response_write_addr, write_addr);
      break;
    }
    write_addr += 32;
  }
}

bool configure(libusb_device_handle *h)
{
  printf("starting configuration...\n");
  uint8_t tx_msg[64] = {0};
  tx_msg[0] =  2; // command: fpga
  tx_msg[1] =  1; // subcmd: configure
  int num_transferred = 0; 
  int tx_rc = libusb_bulk_transfer(h, 2, tx_msg, sizeof(tx_msg),
                                   &num_transferred, 100);
  if (tx_rc != 0)
  {
    printf("tx_rc = %d\n", tx_rc);
    return false;
  }
  uint8_t rx_msg[64] = {0};
  int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                   rx_msg, sizeof(rx_msg),
                                   &num_transferred, 5000);
  if (rx_rc == LIBUSB_ERROR_TIMEOUT)
  {
    printf("timed out. bummer.\n");
    return false;
  }
  else if (rx_rc != 0)
  {
    printf("rx err code: %d\n", rx_rc);
    printf("errno: %d = %s\n", errno, strerror(errno));
    return false;
  }
  printf("configured successfully.\n");
  return true;
}

bool poe_power(libusb_device_handle *h, int on)
{
  printf("turning poe power %d\n", on);
  uint8_t tx_msg[64] = {0};
  tx_msg[0] = 3;
  tx_msg[1] = (uint8_t)on;
  int num_transferred = 0; 
  int tx_rc = libusb_bulk_transfer(h, 2, tx_msg, sizeof(tx_msg),
                                   &num_transferred, 100);
  if (tx_rc != 0)
  {
    printf("tx_rc = %d\n", tx_rc);
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  if (argc < 2)
    return usage();
  const char *cmd = argv[1];
  perish_if(libusb_init(NULL) < 0, "Couldn't init libusb");
  libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, 
                                                            VENDOR_ID, 
                                                            PRODUCT_ID);
  perish_if(h == NULL, "couldn't find device");
  perish_if(0 != libusb_claim_interface(h, 0), "couldn't claim interface");
  printf("device opened successfully.\n");
  signal(SIGINT, signal_handler);
  if (!strcmp(cmd, "dump"))
    dump(h);
  else if (!strcmp(cmd, "burn") && argc >= 3)
    burn(h, argv[2]);
  else if (!strcmp(cmd, "configure"))
    configure(h);
  else if (!strcmp(cmd, "poe") && argc >= 3)
    poe_power(h, atoi(argv[2]));
  libusb_exit(NULL);
  return 0;
}

