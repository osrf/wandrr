#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <errno.h>

#define VENDOR_ID  0xf055
#define PRODUCT_ID 0x0125

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
  puts("usage: menc COMMAND [OPTIONS]\n");
  puts("  where COMMAND is one of { stream }\n");
  return 1;
}

void stream(libusb_device_handle *h)
{
  FILE *f = fopen("log.txt", "w");
  while (!g_done)
  {
    uint8_t rx_msg[64] = {0};
    int nrx = 0;
    int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                     rx_msg, sizeof(rx_msg),
                                     &nrx, 10);
    if (rx_rc == LIBUSB_ERROR_TIMEOUT)
    {
      continue;
      //printf("timeout\n");
      //break;
    }
    else if (rx_rc != 0)
    {
      printf("rx err code: %d\n", rx_rc);
      printf("errno: %d = %s\n", errno, strerror(errno));
      break;
    }

    if (nrx == 64 && rx_msg[0] == 0)
    {
      uint32_t t = 0;
      memcpy(&t, &rx_msg[4], sizeof(uint32_t));
      float angle[3], vel[3];
      uint16_t raw_angle, num_samp;
      for (int i = 0; i < 3; i++)
      {
        angle[i] = *((float *)(rx_msg +  8 + i * 8));
        vel[i]   = *((float *)(rx_msg + 12 + i * 8));
      }
      memcpy(&raw_angle, &rx_msg[32], 2);
      memcpy(&num_samp, &rx_msg[34], 2);
      static bool init_complete = false;
      static int count = 0;
      if (init_complete && count++ % 100 == 0)
      {
        printf("%10.3f %10.3f %10.3f   %10.3f  %10.3f  %10.3f  %8d\n", 
               vel[0], vel[1], vel[2], 
               angle[0], angle[1], angle[2], raw_angle);
        /*
        fprintf(f, "%12d %.6f %.6f %.6f %.6f %.6f %.6f ",
                t, angle[0], angle[1], angle[2],
               vel[0], vel[1], vel[2]);
        fprintf(f, "%d %d %d %.3f\n", 
                raw_angle, num_samp, halls, therm_celsius);
        */
      }
      else
        init_complete = true;
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

