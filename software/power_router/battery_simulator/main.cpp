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
  printf("usage:  battery_simulator CAN_DEVICE\n");
  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);
  if (argc != 2)
    usage();
  const char *serial_device = argv[1];
  LightweightSerial *port = new LightweightSerial(serial_device, 3000000);
  perish_if(!port, "could't open the specified serial port");
  uint8_t b = 0;
  termios old_attrs, new_attrs;
  tcgetattr(fileno(stdin), &old_attrs);
  memcpy(&new_attrs, &old_attrs, sizeof(new_attrs));
  new_attrs.c_lflag &= ~(ECHO | ICANON);
  new_attrs.c_cc[VTIME] = 0;
  new_attrs.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_attrs);
  port->write_cstr("S5\r"); // put it to 250 kbit
  usleep(100000);
  port->write_cstr("O\r"); // open the channel
  usleep(100000);
  port->write_cstr("E\r"); // flush tx buffers
  usleep(100000);
  printf("press 'q' to quit...\n");
  float voltage = 39.8; // starting voltage
  float current = 1.23; // and current 
  while (port->is_ok() && !g_done)
  {
    int c = fgetc(stdin);
    if (c < 0)
    {
      usleep(100000);
    }
    else if (c == 'q')
      break;
    uint32_t id = 0x0000000; // source address = 0 ?
    uint32_t info_id = id | (0xff21 << 8);
    voltage -= 0.05;
    current += 0.05;
    uint16_t voltage_u16 = (uint16_t)(voltage * 20); /// 0.05);
    uint16_t current_u16 = (uint16_t)((current + 1600) * 20);
    uint8_t  temp = 20 + 40; // 20c with offset of -40
    uint8_t info_bytes[8];
    char info_msg[50];
    info_bytes[0] = current_u16 & 0xff;
    info_bytes[1] = current_u16 >> 8;
    info_bytes[2] = voltage_u16 & 0xff;
    info_bytes[3] = voltage_u16 >> 8;
    info_bytes[4] = temp;
    info_bytes[5] = 0;
    info_bytes[6] = 0;
    /*
    printf("voltage = %.2f u16 = %d 0x%02x 0x%02x\n", 
           voltage, voltage_u16, 
           (unsigned)info_bytes[0], (unsigned)info_bytes[1]);
    */
    snprintf(info_msg, sizeof(info_msg), "T%08x7", info_id);
    for (int i = 0; i < 7; i++)
    {
      char hex_byte[10];
      snprintf(hex_byte, sizeof(hex_byte), "%02x", info_bytes[i]);
      strncat(info_msg, hex_byte, 2);
    }
    strncat(info_msg, "\r", 2);
    port->write_cstr(info_msg);
    printf("%s\n", info_msg);

    /*
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
    */
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
  port->write_cstr("C\r"); // close device
  usleep(100000);
  return 0;
}

