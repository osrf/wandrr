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

int main(int argc, char **argv)
{
  perish_if(libusb_init(NULL) < 0, "Couldn't init libusb");
  libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, 
                                                            VENDOR_ID, 
                                                            PRODUCT_ID);
  perish_if(h == NULL, "couldn't find device");
  perish_if(0 != libusb_claim_interface(h, 0), "couldn't claim interface");
  printf("device opened successfully.\n");
  signal(SIGINT, signal_handler);
  uint32_t test_msg = 0;
  uint8_t long_msg[64] = {0};
  while (!g_done)
  {
    test_msg++;
    int num_transferred = 0;
    memcpy(long_msg, &test_msg, sizeof(test_msg));
    int rc = libusb_bulk_transfer(h, 2, long_msg, sizeof(long_msg), 
                                  &num_transferred, 100);
    if (rc != 0)
      printf("tx rc = %d\n", rc);
    for (int i = 0; i < 1; i++)
    {
      uint8_t rx_msg[64] = {0};
      int rx_rc = libusb_bulk_transfer(h, 0x81, 
                                       rx_msg, sizeof(rx_msg),
                                       &num_transferred,
                                       1000);
      if (rx_rc == 0)
        printf("rx %d bytes\r\n", num_transferred);
      else
      {
        printf("errno: %d\n", errno);
        printf("rx err code: %d\r\n", rx_rc);
        usleep(1000000);
      }
    }
  }
  libusb_exit(NULL);
  return 0;
}

