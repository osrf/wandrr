#ifndef STEPPR_INTERFACE_H
#define STEPPR_INTERFACE_H

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>


namespace wandrr
{

class Wandrr;

static const int STEPPR_INTERFACE_NUM_SENSOR_SLOTS = 7;

struct StepprJointState
{
  uint8_t  frame_format;
  uint16_t time_offset;
  uint8_t  frame_len;
  uint8_t  joint_state_format; // 0
  uint8_t  sensor_slot_ids[STEPPR_INTERFACE_NUM_SENSOR_SLOTS]; // 1-7
  uint32_t time_us;
  float    currents[2];
  float    control_target;
  float    menc;
  float    menc_vel;
  float    jenc;
  float    jenc_vel;
  uint32_t last_control_id;
  uint16_t sensor_slots[STEPPR_INTERFACE_NUM_SENSOR_SLOTS];
  uint8_t  csum;
  uint8_t  unused_padding[1]; // 63
} __attribute__((packed));

struct StepprMegaState
{
  uint64_t         router_start_time_us;
  uint64_t         router_tx_time_us;
  StepprJointState joints[15];
  int16_t          router_adc[8];
  float            xsens_accel[3];
  float            xsens_gyro[3];
  float            xsens_mag[3];
  float            xsens_q[4];
  uint16_t         xsens_sample_count;
  uint16_t         xsens_checksum;
} __attribute__((packed));

struct StepprJointCmd
{
  uint8_t  control_mode;
  float    targets[3];
  uint32_t control_id;
} __attribute__((packed));

struct StepprMegaCmd
{
  StepprJointCmd joint_cmds[15];
} __attribute__((packed));

class StepprInterface
{
public:
  int tx_sock_, rx_sock_;
  int tx_count_; // used for multiplexing "slow-sensor" fields
  sockaddr_in mcast_addr_;

  StepprInterface(const char *iface);
  ~StepprInterface() { } // todo

  void tx(Wandrr *w, double secs_since_powerup);

  StepprMegaState ms;
  const static int NUM_JOINTS = 15;
  const static int SENSOR_SLOT_ID_HALLS         = 0;
  const static int SENSOR_SLOT_ID_RAW_MENC      = 1;
  const static int SENSOR_SLOT_ID_RAW_PHASE_A   = 2;
  const static int SENSOR_SLOT_ID_RAW_PHASE_B   = 3;
  const static int SENSOR_SLOT_ID_RAW_PHASE_C   = 4;
  const static int SENSOR_SLOT_ID_CONTROL_MODE  = 5;
  const static int SENSOR_SLOT_ID_RAW_THERM     = 6;
  const static int SENSOR_SLOT_ID_MCB_THERM     = 7;
  const static int SENSOR_SLOT_ID_BUS_VOLTAGE   = 8;
  const static int SENSOR_SLOT_ID_EFFORT_D      = 9;
  const static int SENSOR_SLOT_ID_EFFORT_Q      = 10;
  const static int SENSOR_SLOT_ID_PRESSURE_0    = 11;
  const static int SENSOR_SLOT_ID_PRESSURE_1    = 12;
  const static int SENSOR_SLOT_ID_PRESSURE_2    = 13;
  const static int SENSOR_SLOT_ID_PRESSURE_3    = 14;
  const static int SENSOR_SLOT_ID_MOTOR_CELSIUS = 29;
};

}

#endif
