#include "wandrr_ros.h"
#include "yaml-cpp/yaml.h"
#include <boost/function.hpp>
using namespace wandrr;

static ros::Time t_last_print;

void joint_state_cb(Wandrr *w, int chain, int hop, const JointState * const js)
{
  if (w->logging_)
    w->logState(chain, hop, js);

  static int callback_count = 0; // don't hit ros::Time::now() too often
  if (callback_count++ % 100 == 0)
  {
    ros::Time t(ros::Time::now());
    if ((t - t_last_print).toSec() > 0.1)
    {
      t_last_print = t;
      w->printState();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "play_scripted_currents");
  if (argc < 3)
  {
    ROS_FATAL("syntax: play_scripted_currents PARAMFILE SCRIPTFILE");
    return 1;
  }
  YAML::Node script = YAML::LoadFile(argv[2]);
  WandrrRos wr(argc, argv);
  if (!wr.w->initFromParamFile(argv[1]))
  {
    printf("couldn't configure motor network from configuration file\n");
    return 1;
  }
  wr.w->setJointStateCallback(boost::bind(joint_state_cb, wr.w, _1, _2, _3));
  // turn on all the chains and nodes
  for (size_t chain = 0; chain < wr.w->chains_.size(); chain++)
  {
    int chain_length = wr.w->chains_[chain]->regs.size();
    // start high-speed streaming, from the end of the chain forwards
    for (int hop = chain_length-1; hop >= 0; hop--)
      wr.w->setTXInterval(chain, hop, 1); // full speed ahead
    for (int hop = 0; hop < chain_length; hop++)
    {
      wr.w->setUSBPower(chain, hop, 1);
      wr.w->setLED(chain, hop, true);
      wr.w->setPWMParameters(chain, hop);
      wr.w->setFOCTarget(chain, hop, 0); // target of zero amps to start
      wr.w->setFOCParameters(chain, hop);
      wr.w->setFOCEnable(chain, hop, true);
      wr.w->setEnableFETs(chain, hop, true);
    }
  }
  double t_start = ros::Time::now().toSec();
  float current = 0;
  int c = 0, h = 0;
  double t_prev = 0;
  int control_id = 0;
  float damping = 0;
  for (size_t i = 0; i < script.size(); i++)
  {
    const double t_next = script[i]["t"].as<double>();
    int c_next = script[i]["c"].as<int>();
    int h_next = script[i]["h"].as<int>();
    float damping_next = damping;
    if (script[i]["d"])
      damping_next = script[i]["d"].as<float>();
    float current_next = script[i]["i"].as<float>();
    for (double t = 0; 
        t < t_next && !wr.w->done_; 
        t = ros::Time::now().toSec() - t_start)
    {
      ros::spinOnce();
      wr.w->listen(0.001);
      if (t - t_prev > 0.01) // send every 25 ms to avoid timeout
      {
        t_prev = t;
        wr.w->setMotionCommand(c, h, 3, current, damping, control_id++);
        //wr.w->setFOCTarget(c, h, current);
        //printf("%.6f target tx\n", t);
      }
    }
    h = h_next;
    c = c_next;
    damping = damping_next;
    current = current_next;
    //ROS_INFO("%.6f sending node %d = %.3f amps, damp = %.3f", 
    //         t_prev, h, current, damping);
    //wr.w->setFOCTarget(c, h, current); 
    wr.w->setMotionCommand(c, h, 3, current, damping, control_id++);
  }
  for (size_t chain = 0; chain < wr.w->chains_.size(); chain++)
  {
    int chain_length = wr.w->chains_[chain]->regs.size();
    for (int hop = 0; hop < chain_length; hop++)
    {
      wr.w->setFOCTarget(chain, hop, 0); 
      wr.w->setLED(chain, hop, false);
      wr.w->setEnableFETs(chain, hop, false);
      wr.w->setFOCEnable(chain, hop, false);
      wr.w->setTXInterval(chain, hop, 4883); // about 4 Hz
    }
  }
  return 0;
}
