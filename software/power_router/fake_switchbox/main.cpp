#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "lightweightserial.h"
#include <signal.h>
#include <unistd.h>
#include <termios.h>

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

void usage()
{
  printf("usage:  fake_switchbox DEVICE\n");
  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);
  if (argc != 2)
    usage();
  const char *serial_device = argv[1];
  LightweightSerial *port = new LightweightSerial(serial_device, 115200);
  perish_if(!port, "could't open the specified serial port");
  uint8_t b = 0;
  termios old_attrs, new_attrs;
  tcgetattr(fileno(stdin), &old_attrs);
  memcpy(&new_attrs, &old_attrs, sizeof(new_attrs));
  new_attrs.c_lflag &= ~(ECHO | ICANON);
  new_attrs.c_cc[VTIME] = 0;
  new_attrs.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_attrs);
  printf("press 'q' to quit...\n");
  bool motor_switch = false;
  while (port->is_ok() && !g_done)
  {
    int c = fgetc(stdin);
    if (c < 0)
    {
      usleep(10000);
    }
    else if (c == 'q')
      break;
    else if (c == '1')
    {
      printf("setting motor switch to 'off'\n");
      motor_switch = false;
    }
    else if (c == '2')
    {
      printf("setting motor switch to 'on'\n");
      motor_switch = true;
    }
    if (motor_switch)
      port->write((uint8_t) 0x69);
    else
      port->write((uint8_t) 0xc3);
    //port->write((uint8_t)0x42);
    /*
    if (port->read(&b))
    {
      putc(b, stdout);
      fflush(stdout);
    }
    else
      usleep(1000);
    */
  }
  tcsetattr(fileno(stdin), TCSANOW, &old_attrs);
  return 0;
}

