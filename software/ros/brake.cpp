#include "wandrr_ros.h"
#include <signal.h>
#include <boost/function.hpp>
using namespace wandrr;

static FILE *f = NULL;
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
  ros::init(argc, argv, "wandrr_brake");
  WandrrRos wr(argc, argv);
  if (argc < 2)
  {
    ROS_FATAL("syntax: brake PARAM_FILE");
    return 1;
  }
  if (!wr.w->initFromParamFile(argv[1]))
  {
    ROS_FATAL("couldn't configure motor network from configuration file");
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
      wr.w->setUSBPower(chain, hop, 7);
      wr.w->setLED(chain, hop, true);
      wr.w->setPWMParameters(chain, hop);
      wr.w->setPWMDuty(chain, hop, 0, 0, 0);
      wr.w->setFOCEnable(chain, hop, false);
      ros::Duration(0.001).sleep();
      wr.w->setEnableFETs(chain, hop, true);
    }
  }
  t_last_print = ros::Time::now();

  while (!wr.w->done_)
  {
    wr.w->listen(0.001);
    ros::spinOnce();
  }
  ROS_INFO("exiting...");

  for (size_t chain = 0; chain < wr.w->chains_.size(); chain++)
  {
    int chain_length = wr.w->chains_[chain]->regs.size();
    for (int hop = 0; hop < chain_length; hop++)
    {
      wr.w->setLED(chain, hop, false);
      wr.w->setEnableFETs(chain, hop, false);
      wr.w->setTXInterval(chain, hop, 4883); // about 4 Hz
    }
  }

  return 0;
}
