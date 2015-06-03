#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <errno.h>

#define VENDOR_ID  0xf055
#define PRODUCT_ID 0x0126

//#define LOG

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
  puts("usage: foot COMMAND [OPTIONS]\n");
  puts("  where COMMAND is one of { stream }\n");
  return 1;
}

void stream(libusb_device_handle *h)
{
#ifdef LOG
  FILE *f = fopen("log.txt", "w");
#endif
  while (!g_done)
  {
    uint8_t rx_msg[64] = {0};
    int nrx = 0;
    int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                     rx_msg, sizeof(rx_msg),
                                     &nrx, 10);
    if (rx_rc == LIBUSB_ERROR_TIMEOUT)
      continue;
    else if (rx_rc != 0)
    {
      printf("rx err code: %d\n", rx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }

    if (nrx == 64 && rx_msg[0] == 0)
    {
      uint16_t sample_count = 0;
      memcpy(&sample_count, &rx_msg[2], 2);
      uint32_t t = 0;
      memcpy(&t, &rx_msg[4], sizeof(uint32_t));
      int16_t adc[16] = {0};
      memcpy(&adc, &rx_msg[8], 16*2);

      static int count = 0;
      if (++count % 100 == 0)
      {
        printf("%6d %8d ", sample_count, t);
        for (int i = 0; i < 16; i++)
          printf("%8d ", adc[i]);
        printf("\n");
      }

#ifdef LOG
      fprintf(f, "%d %d ", t, sample_count);
      for (int i = 0; i < 16; i++)
        fprintf(f, "%d ", adc[i]);
      fprintf(f, "\n");
#endif

    }
  }
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
  perish_if(h == NULL, "couldn't find or open device. check permissions?");
  perish_if(0 != libusb_claim_interface(h, 0), "couldn't claim interface");
  printf("device opened successfully.\n");
  signal(SIGINT, signal_handler);
  if (!strcmp(cmd, "stream"))
    stream(h);
  libusb_exit(NULL);
  return 0;
}

