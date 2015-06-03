#include "steppr_interface.h"
#include "wandrr.h"
#include "joint_state.h"
#include <ros/console.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <arpa/inet.h>
using namespace wandrr;

StepprInterface::StepprInterface(const char *iface)
: tx_count_(0)
{
  const char *robot_mcast_addr = "224.0.0.123"; // parameterize someday?
  tx_sock_     = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_     = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_FATAL_COND(tx_sock_     < 0, "SI: couldn't create socket");
  ROS_FATAL_COND(rx_sock_     < 0, "SI: couldn't create socket");
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(robot_mcast_addr);

  ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1)
  {
    ROS_FATAL("getifaddrs() returned an error");
    return;
  }
  std::string tx_iface_addr;
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    if (!ifa->ifa_addr)
      continue;
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET)
      continue;
    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                    host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;
    ROS_INFO("found address %s on interface %s",
             host, ifa->ifa_name);
    if (!strcmp(ifa->ifa_name, iface))
    {
      ROS_INFO("using %s as the tx interface for IPv4 UDP multicast", host);
      tx_iface_addr = host;
      break;
    }
  }
  freeifaddrs(ifaddr);

  in_addr local_addr;
  local_addr.s_addr = inet_addr(tx_iface_addr.c_str());
  int result = 0;
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF,
                      (char *)&local_addr, sizeof(local_addr));
  ROS_FATAL_COND(result < 0, "couldn't set local interface for udp tx sock");

  /////////////////////////////////////////////////////////////////////
  // set up the rx side of things
  int reuseaddr = 1;
  result = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
                      &reuseaddr, sizeof(reuseaddr));
  ROS_FATAL_COND(result < 0, "couldn't set SO_REUSEADDR on UDP RX socket");
  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  rx_bind_addr.sin_port = htons(11305);
  result = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  ROS_FATAL_COND(result < 0, "couldn't bind rx socket to port 11305");
  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(robot_mcast_addr);
  mreq.imr_interface.s_addr = inet_addr(tx_iface_addr.c_str());
  result = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      &mreq, sizeof(mreq));
  ROS_FATAL_COND(result < 0, "couldn't add to multicast group");

  /////////////////////////////////////////////////////////////////////
  // set up our state model's data structures
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    StepprJointState *js = &ms.joints[i];
    js->frame_format = 1; // ?
    js->time_offset = 0;
    js->frame_len = 64;
    js->joint_state_format = 2; // not sure.. might need to be one
    js->sensor_slot_ids[0] = SENSOR_SLOT_ID_RAW_PHASE_A;
    js->sensor_slot_ids[1] = SENSOR_SLOT_ID_RAW_PHASE_B;
    js->sensor_slot_ids[2] = SENSOR_SLOT_ID_RAW_PHASE_C;
    js->sensor_slot_ids[3] = SENSOR_SLOT_ID_EFFORT_D;
    js->sensor_slot_ids[4] = SENSOR_SLOT_ID_EFFORT_Q;
    js->sensor_slot_ids[5] = SENSOR_SLOT_ID_MOTOR_CELSIUS;
    js->sensor_slot_ids[6] = SENSOR_SLOT_ID_RAW_MENC;
    js->time_us = 0;
    js->currents[0] = 0;
    js->currents[1] = 0;
    js->control_target = 0;
    js->menc = 0;
    js->menc_vel = 0;
    js->jenc = 0;
    js->jenc_vel = 0;
    js->last_control_id = 0;
    for (int i = 0; i < STEPPR_INTERFACE_NUM_SENSOR_SLOTS; i++)
      js->sensor_slots[i] = 0;
    js->csum = 0;
    js->unused_padding[0] = 0;
  }
  ms.router_start_time_us = 0;
  ms.router_tx_time_us = 0;
  for (int i = 0; i < 8; i++)
    ms.router_adc[i] = 0;
  for (int i = 0; i < 3; i++)
  {
    ms.xsens_accel[i] = 0;
    ms.xsens_gyro[i] = 0;
    ms.xsens_mag[i] = 0;
  }
  for (int i = 0; i < 4; i++)
    ms.xsens_q[i] = 0;
  ms.xsens_sample_count++;
  ms.xsens_checksum = 0;
  
}

/*
void StepprInterface::updateJointState(int chain, int hop, 
                                       const JointState *js)
{
  int joint_idx = chain * 6 + hop;
  if (joint_idx > NUM_JOINTS)
    return; // adios amigo
  StepprJointState *sjs = &joints[joint_idx];
  sjs->sensor_slots[0] = 
}
*/

// 53000 
static float therm_celsius(uint16_t raw_adc_ticks)
{
  raw_adc_ticks += 32768;
  const float VA = 5.0f;
  const float therm_volts = raw_adc_ticks * VA / 65535.0f;
  const float R1 = 10000.0f, R2 = 10000.0f;
  //const float therm_r_meas = (VA*(R2+R4) - VB*(R1+R2+R4)) / (VB - VA);
  const float therm_r_meas = ((R1 + R2) * therm_volts - R2 * VA) / 
                             (VA - therm_volts);
  //printf("adc ticks = %d therm volts = %.2f resistance = %.0f\n", raw_adc_ticks, therm_volts, therm_r_meas);
  if (therm_r_meas < 0)
    return -99; // brrrr that's cold
  const float THERM_B = 4288.0f;
  const float THERM_R_25C = 50000.0f;
  if (therm_volts > 0.01f)
  {
    // ewww math, how gross
    float c = THERM_B / logf(therm_r_meas /
                             (THERM_R_25C * expf(-THERM_B / 298.15f))) -
              273.15f; // last subtraction is to convert kelvin to celsius
    static float c_filt = -100;
    if (c_filt == -100)
      c_filt = c;
    else
      c_filt = 0.95 * c_filt + 0.05 * c;
    return c_filt;
  }
  else return 0;
}


void StepprInterface::tx(Wandrr *w, double secs_since_powerup)
{
  // dig through the Wandrr model, grab a state snapshot, and send it in
  // STEPPR "megastate" format
  tx_count_++;

  for (int i = 0; i < 15; i++)
  {
    const int c = i / 6; // chain index
    const int n = i % 6; // node index

    StepprJointState *sjs = &ms.joints[i];
    JointState *wjs = &w->ms_.joints[i];
    sjs->time_us = (uint32_t)(wjs->fpga_time & 0xffffffff);
    sjs->currents[0] = wjs->foc_d;
    sjs->currents[1] = wjs->foc_q;
    sjs->control_target = wjs->target_current;
    sjs->menc     = wjs->menc_angle;
    sjs->menc_vel = wjs->menc_vel;
    sjs->jenc     = wjs->jenc_angle_0;
    sjs->jenc_vel = wjs->jenc_vel_0;
    sjs->last_control_id = wjs->control_id;

    // stuff the slow-sensor fields based on the tx count
    if (tx_count_ % 2 == 0)
    {
      sjs->sensor_slot_ids[0] = SENSOR_SLOT_ID_RAW_PHASE_A;
      sjs->sensor_slot_ids[1] = SENSOR_SLOT_ID_RAW_PHASE_B;
      sjs->sensor_slot_ids[2] = SENSOR_SLOT_ID_RAW_PHASE_C;
      sjs->sensor_slot_ids[3] = SENSOR_SLOT_ID_EFFORT_D;
      sjs->sensor_slot_ids[4] = SENSOR_SLOT_ID_EFFORT_Q;
      sjs->sensor_slot_ids[5] = SENSOR_SLOT_ID_MOTOR_CELSIUS;
      sjs->sensor_slot_ids[6] = SENSOR_SLOT_ID_RAW_MENC;

      sjs->sensor_slots[0] = wjs->adc[0];
      sjs->sensor_slots[1] = wjs->adc[1];
      sjs->sensor_slots[2] = wjs->adc[2];
      sjs->sensor_slots[3] = (int16_t)(wjs->effort_d * 100.0f);
      sjs->sensor_slots[4] = (int16_t)(wjs->effort_q * 100.0f);
      if (i != 4)
        sjs->sensor_slots[5] = (uint16_t)(wjs->motor_celsius * 100.0f);
      else
      {
        static float broken_temperature_estimate = 0;
        static int broken_thermistor_count = 0;
        if (++broken_thermistor_count % 100 == 0)
        {
          static const int NODE_WITH_FOOT_THERM = 5; // right ankle
          static const int FOOT_THERM_IDX = 2; // workaround broken menc PCB
          JointState *therm_joint = &w->ms_.joints[NODE_WITH_FOOT_THERM];
          broken_temperature_estimate = therm_celsius(therm_joint->foot_pressures[FOOT_THERM_IDX]);
        }
        sjs->sensor_slots[5] = (uint16_t)(broken_temperature_estimate * 100.0f);
      }
      sjs->sensor_slots[6] = wjs->menc_raw;
    }
    else
    {
      sjs->sensor_slot_ids[0] = SENSOR_SLOT_ID_PRESSURE_0;
      sjs->sensor_slot_ids[1] = SENSOR_SLOT_ID_PRESSURE_1;
      sjs->sensor_slot_ids[2] = SENSOR_SLOT_ID_PRESSURE_2;
      sjs->sensor_slot_ids[3] = SENSOR_SLOT_ID_PRESSURE_3;
      sjs->sensor_slot_ids[4] = SENSOR_SLOT_ID_HALLS;
      sjs->sensor_slot_ids[5] = SENSOR_SLOT_ID_EFFORT_D;
      sjs->sensor_slot_ids[6] = SENSOR_SLOT_ID_EFFORT_Q;

      // grab the foot data from the robot, if has them
      if (c < 2 && n > 1) // spread the foot pad data across 4 joints
      {
        static const int NODE_WITH_FOOT = 5;
        JointState *foot_joint = &w->ms_.joints[c*6 + NODE_WITH_FOOT];
        int foot_sensor_offset = (n - 2) * 4;

        for (int j = 0; j < 4; j++)
        {
          int32_t raw = foot_joint->foot_pressures[foot_sensor_offset + j];
          sjs->sensor_slots[j] = (uint16_t)(raw + 32768);
        }
      }
      else
      {
        for (int j = 0; j < 4; j++)
          sjs->sensor_slots[j] = 0;
      }

      sjs->sensor_slots[4] = wjs->halls;
      sjs->sensor_slots[5] = (int16_t)(wjs->effort_d * 100.0f);
      sjs->sensor_slots[6] = (int16_t)(wjs->effort_q * 100.0f);
    }
  }

  // stuff some power-router battery data into unused fields from steppr
  PowerRouterState *prs = &w->ms_.prs;
  ms.joints[0].sensor_slots[0] = (uint16_t)(prs->battery_voltage[0] * 100.0f);
  ms.joints[0].sensor_slots[1] = (uint16_t)(prs->battery_voltage[1] * 100.0f);
  ms.joints[0].sensor_slots[2] = ( int16_t)(prs->battery_current[0] * 100.0f);
  ms.joints[0].sensor_slots[3] = ( int16_t)(prs->battery_current[1] * 100.0f);
  ms.joints[1].sensor_slots[0] = (uint16_t)prs->battery_celsius[0];
  ms.joints[1].sensor_slots[1] = (uint16_t)prs->battery_celsius[1];
         
  // now that we have copied over the bulk of the data, let's 
  // swap some encoder data around so it all fits into STEPPR formats

  ms.joints[2].jenc     = w->ms_.joints[1].jenc_angle_0;
  ms.joints[2].jenc_vel = w->ms_.joints[1].jenc_vel_0;

  ms.joints[0].jenc     = w->ms_.joints[2].jenc_angle_0;
  ms.joints[0].jenc_vel = w->ms_.joints[2].jenc_vel_0;

  ms.joints[3].jenc     = w->ms_.joints[3].jenc_angle_0;
  ms.joints[3].jenc_vel = w->ms_.joints[3].jenc_vel_0;

  ms.joints[1].jenc     = w->ms_.joints[3].jenc_angle_1;
  ms.joints[1].jenc_vel = w->ms_.joints[3].jenc_vel_1;

  ms.joints[4].jenc     = w->ms_.joints[4].jenc_angle_1;
  ms.joints[4].jenc_vel = w->ms_.joints[4].jenc_vel_1;

  ms.joints[5].jenc     = w->ms_.joints[5].jenc_angle_0;
  ms.joints[5].jenc_vel = w->ms_.joints[5].jenc_vel_0;


  ms.joints[2+6].jenc     = w->ms_.joints[1+6].jenc_angle_0;
  ms.joints[2+6].jenc_vel = w->ms_.joints[1+6].jenc_vel_0;

  ms.joints[0+6].jenc     = w->ms_.joints[2+6].jenc_angle_0;
  ms.joints[0+6].jenc_vel = w->ms_.joints[2+6].jenc_vel_0;

  ms.joints[3+6].jenc     = w->ms_.joints[3+6].jenc_angle_0;
  ms.joints[3+6].jenc_vel = w->ms_.joints[3+6].jenc_vel_0;

  ms.joints[1+6].jenc     = w->ms_.joints[3+6].jenc_angle_1;
  ms.joints[1+6].jenc_vel = w->ms_.joints[3+6].jenc_vel_1;

  ms.joints[4+6].jenc     = w->ms_.joints[4+6].jenc_angle_0;
  ms.joints[4+6].jenc_vel = w->ms_.joints[4+6].jenc_vel_0;

  ms.joints[5+6].jenc     = w->ms_.joints[5+6].jenc_angle_0;
  ms.joints[5+6].jenc_vel = w->ms_.joints[5+6].jenc_vel_0;

  ms.joints[1+12].jenc     = w->ms_.joints[1+12].jenc_angle_1;
  ms.joints[1+12].jenc_vel = w->ms_.joints[1+12].jenc_vel_1;

  for (int i = 0; i < 15; i++)
  {
    StepprJointState *sjs = &ms.joints[i];
    sjs->csum = 0;
    for (uint8_t b = 4; b < 62; b++)
      sjs->csum += ((uint8_t *)sjs)[b];
  }

  ms.router_start_time_us = secs_since_powerup * 1e6;
  ms.router_tx_time_us = secs_since_powerup * 1e6;

  // todo: power-router ADC's 
  // TODO: send UDP mcast packet to same group as steppr

  mcast_addr_.sin_port = htons(11303);
  int nsent = sendto(tx_sock_, &ms, sizeof(StepprMegaState), 0,
                     (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
}
