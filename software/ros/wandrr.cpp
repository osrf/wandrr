#include "wandrr.h"
#include "outbound_packet.h"
#include "joint_state.h"
#include <errno.h>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <ros/ros.h> // for ros::Time
#include <sys/stat.h>
#include <sys/types.h>
#include "yaml-cpp/yaml.h"
#include <arpa/inet.h>
using namespace wandrr;
using std::string;

Wandrr::Wandrr()
: done_(false),
  logging_(false),
  steppr_rx_sock_(-1),
  sr_(NULL),
  pr_(NULL)
{
}

Wandrr::~Wandrr()
{
  // todo, if it ever matters
}

void Wandrr::tx(const int chain, const OutboundPacket &obp)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->tx(obp);
}

void Wandrr::perish_if(bool b, const char *msg)
{
  if (b)
  {
    printf("%s\n", msg);
    exit(1);
  }
}

void Wandrr::resetOverheatLatch(const int chain, const int hop)
{
  if (chain >= chains_.size())
  {
    printf("illegal chain: %d\n", chain);
    return;
  }
  chains_[chain]->resetOverheatLatch(hop);
}

void Wandrr::setMotionCommand(const int chain, const int hop,
                              const uint8_t control_mode,
                              const float target_current,
                              const float damping,
                              const uint32_t control_id)
{
  if (chain >= chains_.size())
  {
    printf("illegal chain: %d\n", chain);
    return;
  }
  chains_[chain]->setMotionCommand(hop, control_mode,
                                   target_current, damping, control_id);
}

void Wandrr::setLED(const int chain, const int hop, bool on)
{
  if (chain >= chains_.size())
  {
    printf("illegal chain: %d\n", chain);
    return;
  }
  chains_[chain]->setLED(hop, on);
}

void Wandrr::setTXInterval(const int chain, const int hop, const uint16_t num_adc_ticks)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setTXInterval(hop, num_adc_ticks);
}

void Wandrr::listen(const double max_seconds)
{
  if (chains_.empty())
  {
    printf("no chains\n");
    usleep(1000);
    return; // bogus
  }
  ros::Time t_start(ros::Time::now());
  //printf("wandrr::listen @ %.6f\n", t_start.toSec());
  static uint8_t buf[1500] = {0};
  if (done_)
    printf("wandrr::listen done\n");
  if (!ros::ok())
    printf("wandrr::listen ros::ok() returning false...\n");
  int pass = 0;
  while (!done_ && ros::ok())
  {
    fd_set rdset;
    FD_ZERO(&rdset);
    int max_sock = 0;
    if (steppr_rx_sock_ >= 0)
    {
      FD_SET(steppr_rx_sock_, &rdset);
      max_sock = steppr_rx_sock_;
    }
    if (sr_)
    {
      FD_SET(sr_->rx_sock_, &rdset);
      if (sr_->rx_sock_ > max_sock)
        max_sock = sr_->rx_sock_;
    }
    if (pr_)
    {
      FD_SET(pr_->rx_sock_, &rdset);
      if (pr_->rx_sock_ > max_sock)
        max_sock = pr_->rx_sock_;
    }
    for (size_t chain = 0; chain < chains_.size(); chain++)
    {
      int sock = chains_[chain]->rx_sock_;
      if (sock > max_sock)
        max_sock = sock;
      FD_SET(sock, &rdset);
    }

    ros::Time t(ros::Time::now());
    double elapsed_time = (t - t_start).toSec();
    if (pass++ > 0 && (elapsed_time > max_seconds))
      break; // all done.

    double time_left = max_seconds - elapsed_time;
    if (time_left < 0)
      time_left = 0;

    timeval timeout;
    timeout.tv_sec = (time_t)trunc(time_left);
    timeout.tv_usec = (suseconds_t)((time_left - timeout.tv_sec) * 1e6);

    int rv = select(max_sock+1, &rdset, NULL, NULL, &timeout);
    if (rv < 0)
    {
      ROS_ERROR("abnormal exit from select()");
      return;
    }
    if (rv > 0)
    {
      // find out which socket is ready to read
      if (steppr_rx_sock_ >= 0 && FD_ISSET(steppr_rx_sock_, &rdset))
      {
        sockaddr_in rx_addr;
        socklen_t rx_addr_len = sizeof(rx_addr);
        const int nbytes = recvfrom(steppr_rx_sock_, buf, sizeof(buf), 0, 
                                    (sockaddr *)&rx_addr, &rx_addr_len);
        if (nbytes < 0)
        {
          ROS_ERROR("error in recvfrom");
          break;
        }
        //ROS_INFO("rx %d bytes on steppr_rx_sock_", nbytes);
        if (nbytes == sizeof(StepprMegaCmd))
        {
          if (rx_sock_cb_)
            rx_sock_cb_((StepprMegaCmd *)buf);
        }
        else
          ROS_WARN("unexpected rx len %d on steppr_rx_sock_", nbytes);
      }
      
      if (sr_ && FD_ISSET(sr_->rx_sock_, &rdset))
      {
        const int nbytes = recvfrom(sr_->rx_sock_, buf, sizeof(buf), 0, 
                                    NULL, NULL);
        if (nbytes < 0)
        {
          ROS_ERROR("error in recvfrom");
          break;
        }
        if (nbytes == sizeof(SerialRouterState))
        {
          memcpy(&ms_.srs, buf, sizeof(SerialRouterState));
          if (sr_cb_)
            sr_cb_((SerialRouterState *)buf);
        }
        else
          ROS_WARN("unexpected rx len %d on steppr_rx_sock_", nbytes);
      }

      if (pr_ && FD_ISSET(pr_->rx_sock_, &rdset))
      {
        const int nbytes = recvfrom(pr_->rx_sock_, buf, sizeof(buf), 0, 
                                    NULL, NULL);
        if (nbytes < 0)
        {
          ROS_ERROR("error in recvfrom");
          break;
        }
        if (nbytes == sizeof(PowerRouterState))
        {
          memcpy(&ms_.prs, buf, sizeof(PowerRouterState));
          if (pr_cb_)
            pr_cb_((PowerRouterState *)buf);
        }
        else
          ROS_WARN("unexpected rx len %d on steppr_rx_sock_", nbytes);
      }

      for (size_t chain = 0; chain < chains_.size(); chain++)
      {
        const int sock = chains_[chain]->rx_sock_;
        if (!FD_ISSET(sock, &rdset))
          continue; // boring, nothing ready to read
        sockaddr_in rx_addr;
        socklen_t rx_addr_len = sizeof(rx_addr);
        const int nbytes = recvfrom(sock, buf, sizeof(buf), 0, 
                                    (sockaddr *)&rx_addr, &rx_addr_len);
        if (nbytes < 0)
        {
          ROS_ERROR("error in recvfrom");
          break;
        }
        //printf("rxaddr: %s\n", inet_ntoa(rx_addr.sin_addr));
        const int rx_chain = ((ntohl(rx_addr.sin_addr.s_addr) >> 8) - 0xac) & 0xff;
        //printf("rx %d on chain %d\n", nbytes, rx_chain);
        //printf("rx chain: %d\n", rx_chain);
        if (buf[0] == 0x21 && buf[1] == 0x43) // packet sentinels we expect
        {
          uint16_t read_pos = 4; // skip over the inbound chain hop count
          while (read_pos < nbytes)
          {
            const uint16_t sender_hop_count = buf[read_pos];
            read_pos += 2;
            const uint16_t submsg_len = buf[read_pos]; // just 8 bits for now...
            read_pos += 2;
            const uint8_t msg_id = buf[read_pos];
            if (buf[read_pos] == 0x42 || // a state message, no foot data
                buf[read_pos] == 0x43)   // it's a state message with foot data
            {
              read_pos++;
              JointState *js = (JointState *)&buf[read_pos];
              updateState(rx_chain, sender_hop_count, js, t.toSec());
              if (joint_state_cb_)
                joint_state_cb_(rx_chain, sender_hop_count, js);
              read_pos += submsg_len-1;
            }
            else if (buf[read_pos] == 0x99)
            {
              read_pos++;
              if (mcu_rx_cb_)
                mcu_rx_cb_(rx_chain, sender_hop_count, &buf[read_pos], submsg_len-1);
              //printf("rx submsg len %d\n", submsg_len);
              read_pos += submsg_len-1;
            }
            else if (buf[read_pos] == 0xff)
              break; // we've hit unused space in the packet
            else
              printf("unknown submessage type: 0x%02x\n", buf[read_pos]);
          }
        }
      }
    }
  }
}
 
void Wandrr::setUSBPower(const int chain, const int hop, const uint8_t power_bits)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setUSBPower(hop, power_bits);
}

void Wandrr::setChainParameters(const int chain,
                                const int hop,
                                const bool endpoint, 
                                const uint16_t payload_len)
{
  if (chain >= chains_.size())
  {
    printf("ahhh short chain\n");
    return;
  }
  chains_[chain]->setChainParameters(hop, endpoint, payload_len);
}

void Wandrr::setAddresses(const int chain, const int hop)
{
  if (chain >= chains_.size())
  {
    printf("woah there partner. chain > chain_size: %d > %d\n",
           chain, (int)chains_.size());
    return;
  }
  chains_[chain]->setAddresses(hop);
}

void Wandrr::setPWMParameters(const int chain, const int hop)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setPWMParameters(hop);
}

void Wandrr::setFOCParameters(const int chain, const int hop)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setFOCParameters(hop);
}

void Wandrr::setFootParameters(const int chain, const int hop)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setFootParameters(hop);
}

void Wandrr::setEnableFETs(const int chain, const int hop, const bool enable)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setEnableFETs(hop, enable);
}

void Wandrr::setPWMDuty(const int chain,
                        const int hop, 
                        const uint16_t a, 
                        const uint16_t b, 
                        const uint16_t c)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setPWMDuty(hop, a, b, c);
}

void Wandrr::setFOCTarget(const int chain, const int hop, const float amps)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setFOCTarget(hop, amps);
}

void Wandrr::zeroAllFOCTargets()
{
  for (int c = 0; c < (int)chains_.size(); c++)
    for (int n = 0; n < (int)chains_[c]->regs.size(); n++)
      setMotionCommand(c, n, 3, 0, 0, 0);
      //setFOCTarget(c, n, 0);
}

void Wandrr::setFOCEnable(const int chain, const int hop, const bool enable)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->setFOCEnable(hop, enable);
}

void Wandrr::txMCU(const int chain, const int hop, const uint8_t *data, const uint8_t len)
{
  if (chain >= chains_.size())
    return;
  chains_[chain]->txMCU(hop, data, len);
}

void Wandrr::txMCUFrame(const int chain,
                        const int hop, 
                        const uint8_t *data, const uint8_t len)
{
  if (chain >= chains_.size())
  {
    printf("woah! tried to send to chain %d but there are only %d chains available\n", 
           chain, (int)chains_.size());
    return;
  }
  //ros::Time t_start(ros::Time::now());
  //printf("wandrr::txMCU @ %.6f\n", t_start.toSec());
  chains_[chain]->txMCUFrame(hop, data, len);
}

bool Wandrr::loadParamFile(const char *filename)
{
  ROS_INFO("loading param file [%s]", filename);
  YAML::Node top = YAML::LoadFile(filename);
  YAML::Node chains_yaml = top["chains"];
  YAML::Node globals = top["global"];
  for (size_t i = 0; i < chains_yaml.size(); i++)
  {
    printf("found chain %d\n", (int)i);
    YAML::Node chain_yaml = chains_yaml[i];
    for (YAML::const_iterator it = chain_yaml.begin(); it != chain_yaml.end(); ++it)
    {
      std::string iface = it->first.as<std::string>();
      YAML::Node nodes = it->second; //.as<YAML::Node>();
      printf("  iface = %s\n", iface.c_str());
      // there are assumptions being made about the relationship between the 
      // inbound routing table and the ethernet interface number.
      Chain *c = new Chain(iface.c_str(), i);
      chains_.push_back(c);
      for (size_t n = 0; n < nodes.size(); n++)
      {
        c->addNode();
        MCRegs *regs = &c->regs[n];
        YAML::Node node = nodes[n];
        printf("    node %d\n", (int)n);
        printf("      kp = %.3f\n", node["foc_kp"].as<float>());
        regs->foc_kp = node["foc_kp"].as<float>();
        regs->foc_ki = node["foc_ki"].as<float>();
        regs->foc_stator_offset = node["foc_stator_offset"].as<int>();
        regs->foc_max_effort = node["foc_max_effort"].as<float>();
        regs->foc_integrator_limit = node["foc_integrator_limit"].as<float>();
        regs->foc_bus_voltage = globals["foc_bus_voltage"].as<float>();
        regs->foc_pole_pairs = 8;
        try {
          regs->foc_pole_pairs = node["pole_pairs"].as<int>();
        }
        catch (YAML::Exception &e) { }
        printf("      pole pairs = %d\n", regs->foc_pole_pairs);

        MC *mc = &c->mcs_[n];
        try {
          mc->enable_mosfets = node["enable_mosfets"].as<bool>();
        }
        catch (YAML::Exception &e) { }

        try {
          regs->foc_damping = -1000000.0f * node["damping"].as<float>();
        }
        catch (YAML::Exception &e) { }

        try {
          regs->foc_resistance = -1.0f * node["resistance"].as<float>();
        }
        catch (YAML::Exception &e) { }

        if (node["temperature_limit"])
          regs->foc_temperature_limit = node["temperature_limit"].as<float>();
        else
          regs->foc_temperature_limit = 90.0f; // 90C 

        if (node["ignore_thermistor"])
        {
          if (node["ignore_thermistor"].as<bool>())
          {
            mc->ignore_thermistor = true;
            regs->master |= 0x10; // magic bit to ignore thermal shutdown rules
          }
        }

        if (node["foot_port"])
        {
          mc->has_foot = true;
          regs->foot_control = 1;
          regs->foot_usb_port = node["foot_port"].as<int>();
        }
      }
      break;
    }
    
    std::string serial_router_iface;
    if (globals["serial_router_interface"])
    {
      serial_router_iface = globals["serial_router_interface"].as<std::string>();
      sr_ = new SerialRouter(serial_router_iface.c_str());
    }

    std::string power_router_iface;
    if (globals["power_router_interface"])
    {
      power_router_iface = globals["power_router_interface"].as<std::string>();
      pr_ = new PowerRouter(power_router_iface.c_str());
    }
  }
  return true;
}

void Wandrr::setLogging(const bool on)
{
  ROS_INFO("setlogging()");
  if (on && logs_.empty())
  {
    ROS_INFO("creating log files");
    char dir[80];
    const ros::Time t_ros(ros::Time::now());
    const time_t t = t_ros.sec;
    const struct tm ts = *localtime(&t);
    strftime(dir, sizeof(dir), "%Y%m%d_%H%M%S", &ts);
    mkdir(dir, 0700);
    for (int chain = 0; chain < (int)chains_.size(); chain++)
    {
      std::vector<FILE *> chain_logs;
      for (int node = 0; node < (int)chains_[chain]->regs.size(); node++)
      {
        char fname[160];
        snprintf(fname, sizeof(fname), "%s/%d.%d.txt", dir, chain, node);
        ROS_INFO("creating log %s", fname);
        FILE *f = fopen(fname, "w");
        chain_logs.push_back(f);
      }
      logs_.push_back(chain_logs);
    }
  }
  logging_ = on;
}

void Wandrr::logState(const int chain, const int hop, 
                      const JointState * const js)
{
  if (logs_.empty())
    setLogging(true);
  if (chain >= (int)logs_.size() || hop >= (int)logs_[chain].size())
    return; 
  FILE *f = logs_[chain][hop];
  if (!f)
  {
    ROS_ERROR("unexpected null file pointer in Wandrr::logState");
    return;
  }
  fprintf(f, "%ld %d %d %d %d %.5f %.5f %.3f %.3f %.3f %.9f %.3f %.3f %.6f %.6f %d %.3f %d %.3f\n", 
          js->fpga_time, js->menc_raw, 
          js->adc[0], js->adc[1], js->adc[2], 
          js->foc_d, js->foc_q,
          js->effort_d, js->effort_q,
          js->menc_angle, js->menc_vel,
          js->jenc_angle_0,
          js->jenc_angle_1,
          js->jenc_vel_0,
          js->jenc_vel_1,
          js->halls,
          js->target_current,
          js->control_id,
          js->motor_celsius);
}

bool Wandrr::initFromParamFile(const char *filename)
{
  if (!loadParamFile(filename))
    return false;
  for (size_t chain = 0; chain < chains_.size(); chain++)
  {
    // first, make sure all but the last node are NOT set to be an endpoint
    const int chain_length = chains_[chain]->regs.size();
    const int payload_size = 1400; //50 + 128 * chain_length + 50;
    for (int hop = 0; hop < chain_length-1; hop++)
    {
      setAddresses(chain, hop);
      setChainParameters(chain, hop, false, payload_size);
    }
    // now, set the last node to be the endpoint
    printf("setting chain %d node %d as endpoint...\n", 
           (int)chain, chain_length-1);
    setAddresses(chain, chain_length-1);
    setChainParameters(chain, chain_length-1, true, payload_size);
  }
  return true;
}

void Wandrr::updateState(const int chain, const int hop, 
                         const JointState * const js,
                         const double ros_time)
{
  if (chain > 2 || hop > 6)
    return; // woah
  int i = chain * 6 + hop;
  if (i > MegaState::NUM_JOINTS)
    return; // woah
  ms_.joints[i] = *js; // careful now
  if (chain < chains_.size() && 
      hop < chains_[chain]->mcs_.size())
  {
    MC *mc = &chains_[chain]->mcs_[hop];
    if (js->fpga_time != mc->inbound_data_last_timestamp)
    {
      mc->inbound_data_last_timestamp = js->fpga_time;
      mc->inbound_data_valid = true;
      mc->inbound_data_last_change_time = ros_time;
    }
  }
}

void Wandrr::printState()
{
  printf("\n\n\n\n");
  printf("id      timestamp   mraw h  mang     j0ang   j1ang   i_q     effort_q temp\n");
  printf("=====================================================================\n");
  for (int i = 0; i < MegaState::NUM_JOINTS; i++)
  {
    printf("%d:%d %13lu %6d %d %6.1f %6.1f %6.1f %6.1f %6.1f %4.1f\n", 
           i / 6, // chain
           i % 6, // node index on chain
           ms_.joints[i].fpga_time,
           ms_.joints[i].menc_raw,
           ms_.joints[i].halls,
           ms_.joints[i].menc_angle,
           ms_.joints[i].jenc_angle_0,
           ms_.joints[i].jenc_angle_1,
           ms_.joints[i].foc_q,
           ms_.joints[i].effort_q,
           ms_.joints[i].motor_celsius);
  }
  printf("imu: t = %8d  sample = %8d  accels = [%.2f %.2f %.2f]\n",
           ms_.srs.time_us,
           ms_.srs.imu_sample_counter,
           ms_.srs.accels[0],
           ms_.srs.accels[1],
           ms_.srs.accels[2]);
  for (int i = 0; i < MegaState::NUM_JOINTS; i++)
  {
    int c = i / 6;
    int n = i % 6;
    if (c >= chains_.size())
      continue;
    if (n >= chains_[c]->mcs_.size())
      continue;
    MC *mc = &chains_[c]->mcs_[n];
    if (mc->has_foot)
    {
      printf("foot %d:%d   t = %08d\n  ", c, n, ms_.joints[i].foot_time); 
      for (int j = 0; j < 16; j++)
      {
        printf("%7d ", ((int32_t)ms_.joints[i].foot_pressures[j])+32768);
        if (j % 8 == 7)
          printf("\n  ");
      }
    }
  }
  printf("power channel states: %d %d %d %d %d %d\n",
         ms_.prs.channels_active & 0x01 ? 1 : 0,
         ms_.prs.channels_active & 0x02 ? 1 : 0,
         ms_.prs.channels_active & 0x04 ? 1 : 0,
         ms_.prs.channels_active & 0x08 ? 1 : 0,
         ms_.prs.channels_active & 0x10 ? 1 : 0,
         ms_.prs.channels_active & 0x20 ? 1 : 0);
  printf("power adcs: [ %7d %7d %7d %7d\n",
         ms_.prs.adc[0],
         ms_.prs.adc[1],
         ms_.prs.adc[2],
         ms_.prs.adc[3]);
  printf("              %7d %7d %7d %7d   ]\n",
         ms_.prs.adc[4],
         ms_.prs.adc[5],
         ms_.prs.adc[6],
         ms_.prs.adc[7]);
}


