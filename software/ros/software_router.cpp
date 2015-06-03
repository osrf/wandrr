#include "wandrr_ros.h"
#include "steppr_interface.h"
#include "serial_router.h"
#include "local_mcast_tx.h"
#include "mini_state.h"
#include <sched.h>
using namespace wandrr;

static ros::Time t_last_print, g_t_last_cmd_tx;
static StepprInterface *g_si = NULL;
static SerialRouterState g_serial_router_state;
static PowerRouterState g_power_router_state;
static bool g_quiet = true;
static bool g_tx_only_currents = false;

void joint_state_cb(Wandrr *w, int chain, int hop, const JointState * const js)
{
  if (w->logging_)
    w->logState(chain, hop, js);

  ros::Time t(ros::Time::now());
  if (!g_quiet)
  {
    if ((t - t_last_print).toSec() > 0.1)
    {
      t_last_print = t;
      w->printState();
    }
  }
}

bool g_prev_enabled[15] = {false};

void tx_chain_cmds(Wandrr *w, Chain *c, StepprMegaCmd *cmd, 
                   int start_node, int max_nodes)
{
  const double t = g_t_last_cmd_tx.toSec();
  OutboundPacket pkt;
  int n_set = 0;
  bool joint_must_disable[6] = {false};
  bool joint_must_enable[6] = {false};
  //printf("tx mode: %d  target = %.3f\n",
  //       cmd->joint_cmds[7].control_mode, cmd->joint_cmds[7].targets[0]);
  for (int i = 0; i < max_nodes; i++)
  {
    if (i >= c->regs.size())
      break; // our chain isn't this long
    if (start_node + i >= 15)
      break; // woah there partner
    StepprJointCmd *jc = &cmd->joint_cmds[start_node+i];
    if (jc->control_mode == 3)
    {
      float current_target = jc->targets[0];
      if (i < c->mcs_.size())
      {
        if (!c->mcs_[i].inbound_data_valid)
          current_target = 0; // no valid inbound data. only send zero amps.
        else
        {
          // double-check that it's still OK to send non-zero currents here
          const double dt = t - c->mcs_[i].inbound_data_last_change_time;
          if (dt > 0.1)
          {
            printf("inbound data timeout on joint %d\n", i);
            c->mcs_[i].inbound_data_valid = false;
            current_target = 0;
          }
        }
      }

      c->regs[i].foc_target = current_target;
      if (!g_prev_enabled[start_node+i])
      {
        joint_must_enable[i] = true;
        g_prev_enabled[start_node+i] = true;
      }
    }
    else
    {
      c->regs[i].foc_target = 0;
      if (g_prev_enabled[start_node+i])
      {
        joint_must_disable[i] = true;
        g_prev_enabled[start_node+i] = false;
      }
    }
    if (g_tx_only_currents)
      pkt.appendSetFOCTarget(i, &c->regs[i]);
    else
      pkt.appendMotionCommand(i, 3, 
                              c->regs[i].foc_target, 
                              jc->targets[1],
                              jc->control_id);
    n_set++;
    //printf("set %d to %.3f\n", i, c->regs[i].foc_target);
  }
  if (n_set)
    c->tx(pkt);

  for (int i = 0; i < max_nodes; i++)
  {
    if (joint_must_enable[i])
    {
      c->setFOCEnable(i, true);
      c->setEnableFETs(i, true);
    }
    if (joint_must_disable[i])
    {
      c->setFOCEnable(i, false);
      c->setEnableFETs(i, false);
    }
  }
}

void steppr_rx_cb(Wandrr *w, StepprMegaCmd *cmd)
{
  g_t_last_cmd_tx = ros::Time::now();
  //printf("steppr rx cb\n");
  tx_chain_cmds(w, w->chains_[0], cmd, 0, 6);
  if (w->chains_.size() > 1)
    tx_chain_cmds(w, w->chains_[1], cmd, 6, 6);
  if (w->chains_.size() > 2)
    tx_chain_cmds(w, w->chains_[2], cmd, 12, 3);
}

void power_router_cb(PowerRouterState *state)
{
  StepprMegaState *sms = &g_si->ms; // save typing
  for (int i = 0; i < 8; i++)
    sms->router_adc[i] = state->adc[i];
}

void serial_router_cb(SerialRouterState *state)
{
  StepprMegaState *sms = &g_si->ms; // save typing
  for (int i = 0; i < 3; i++)
  {
    sms->xsens_accel[i] = state->accels[i];
    sms->xsens_gyro[i] = state->gyros[i];
    sms->xsens_mag[i] = state->mags[i];
  }
  for (int i = 0; i < 4; i++)
    sms->xsens_q[i] = state->quaternion[i];
  sms->xsens_sample_count = state->imu_sample_counter;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "software_router");
  if (argc < 2)
  {
    ROS_FATAL("syntax: software_router PARAMFILE");
    return 1;
  }
  WandrrRos wr(argc, argv);
  if (!wr.w->initFromParamFile(argv[1]))
  {
    ROS_FATAL("couldn't load param file. adios amigo.");
    return 1;
  }

  wr.w->setJointStateCallback(boost::bind(joint_state_cb, wr.w, _1, _2, _3));
  wr.w->setSerialRouterCallback(serial_router_cb);
  wr.w->setPowerRouterCallback(power_router_cb);

  std::string network_interface;
  wr.nh_private.param<std::string>("network_interface", network_interface, "eth0");
  g_si = new StepprInterface(network_interface.c_str());
  wr.w->steppr_rx_sock_ = g_si->rx_sock_;
  wr.w->setRxSockCallback(boost::bind(steppr_rx_cb, wr.w, _1));

  LocalMCastTx lmct("224.0.0.149");
  MiniState mini_state;

  bool enable_mosfets = false;
  wr.nh_private.param<bool>("enable_mosfets", enable_mosfets, false);

  bool usb_dont_respond = false;
  wr.nh_private.param<bool>("usb_dont_respond", usb_dont_respond, false);

  wr.nh_private.param<bool>("quiet", g_quiet, true);

  wr.nh_private.param<bool>("tx_only_currents", g_tx_only_currents, false);

  bool tx_mini_state = true;
  wr.nh_private.param<bool>("tx_mini_state", tx_mini_state, true);

  // turn on all the chains and nodes
  for (size_t chain = 0; chain < wr.w->chains_.size(); chain++)
  {
    int chain_length = wr.w->chains_[chain]->regs.size();
    // start high-speed streaming, from the end of the chain forwards
    for (int hop = chain_length-1; hop >= 0; hop--)
      wr.w->setTXInterval(chain, hop, 1); // gimme lots of data plz
    for (int hop = 0; hop < chain_length; hop++)
    {
      wr.w->setUSBPower(chain, hop, 0x1f);
      wr.w->setFootParameters(chain, hop);
      if (usb_dont_respond)
      {
        ROS_INFO("setting usb don't respond to true");
        wr.w->chains_[chain]->setUSBDontRespond(hop, true);
      }
      else
        wr.w->chains_[chain]->setUSBDontRespond(hop, false);
      wr.w->setLED(chain, hop, true);
      wr.w->setPWMParameters(chain, hop);
      //wr.w->setPWMDuty(chain, hop, 0, 0, 0);
      wr.w->setFOCTarget(chain, hop, 0);
      wr.w->setFOCParameters(chain, hop);
      ros::Duration(0.001).sleep();
      //wr.w->setFOCEnable(chain, hop, true);
      ros::Duration(0.001).sleep();
      //if (enable_mosfets)
      //  wr.w->setEnableFETs(chain, hop, true);
    }
  }
  t_last_print = ros::Time::now();

  sched_param param;
  param.sched_priority = 45;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
  {
    perror("sched_setscheduler failed");
    return 1;
  }

  g_t_last_cmd_tx = ros::Time::now();
  ros::Time t_init(ros::Time::now());
  ros::Time t_last_steppr_emulation_tx(t_init);
  ros::Time t_last_mini_state_tx(t_init);

  while (!wr.w->done_)
  {
    ros::Time t(ros::Time::now());

    static const double TX_INTERVAL = 0.001;
    double time_left = TX_INTERVAL - (t - t_last_steppr_emulation_tx).toSec();
    if (time_left < 0)
      time_left = 0;

    wr.w->listen(time_left); //.000001); //0.001);
    //ros::spinOnce();


    const double steppr_dt = (t - t_last_steppr_emulation_tx).toSec();
    if (steppr_dt >= TX_INTERVAL)
    {
      g_si->tx(wr.w, (t - t_init).toSec());
      if (steppr_dt > 100 * TX_INTERVAL)
        t_last_steppr_emulation_tx = t; // big time network hiccup. skip ahead.
      else
        t_last_steppr_emulation_tx += ros::Duration(TX_INTERVAL); // maintain rate
#ifdef NOISY_DEBUG
      if (steppr_dt >= 10 * TX_INTERVAL)
        printf("txdt = %.6f\n", steppr_dt);
#endif
    }

    // mini state @ 100 Hz for arm control and whatever else
    if (tx_mini_state && (t - t_last_mini_state_tx).toSec() > 0.01)
    {
      t_last_mini_state_tx += ros::Duration(0.01);
      for (int i = 0; i < 15; i++)
        mini_state.joint_angles[i] = g_si->ms.joints[i].jenc;
      lmct.tx((uint8_t *)&mini_state, sizeof(mini_state));
    }

    if ((t - g_t_last_cmd_tx).toSec() > 0.1)
    {
      wr.w->zeroAllFOCTargets();
      g_t_last_cmd_tx = t;
    }
  }
  ROS_INFO("exiting...");

  for (size_t chain = 0; chain < wr.w->chains_.size(); chain++)
  {
    int chain_length = wr.w->chains_[chain]->regs.size();
    for (int hop = 0; hop < chain_length; hop++)
    {
      wr.w->setEnableFETs(chain, hop, false);
      wr.w->setFOCTarget(chain, hop, 0);
      wr.w->setLED(chain, hop, false);
      wr.w->setTXInterval(chain, hop, 4883); // about 4 Hz
      wr.w->setFOCEnable(chain, hop, false);
    }
  }
  return 0;
}
