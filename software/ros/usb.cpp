#include "wandrr.h"
#include "wandrr_ros.h"
#include "outbound_packet.h"
#include <cstdio>
#include <boost/bind.hpp>
#include <ros/ros.h>
using namespace wandrr;

static int g_chain = 0;
static int g_node = 0;
static float g_menc_angle   = 0;
static float g_jenc_angle_0 = 0;
static float g_jenc_angle_1 = 0;

void joint_state_cb(Wandrr *w, int chain, int hop, const JointState * const js)
{
  if (g_chain == chain && g_node == hop)
  {
    g_menc_angle   = js->menc_angle;
    g_jenc_angle_0 = js->jenc_angle_0;
    g_jenc_angle_1 = js->jenc_angle_1;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wandrr_usb");
  WandrrRos wr(argc, argv);
  if (argc < 6)
  {
    printf("syntax: usb PARAMFILE COMMAND CHAIN HOP_COUNT [PARAMS]\n");
    return 1;
  }
  if (!wr.w->loadParamFile(argv[1]))
  {
    printf("bogus param file\n");
    return 1;
  }

  const char *cmd = argv[2];
  g_chain = atoi(argv[3]);
  g_node = atoi(argv[4]);

  if (!strcmp(cmd, "power"))
  {
    wr.w->setUSBPower(g_chain, g_node, atoi(argv[5]));
    return 1;
  }
  else if (!strcmp(cmd, "power_cycle_test"))
  {
    const int num_tests = atoi(argv[5]);
    wr.w->setJointStateCallback(boost::bind(joint_state_cb, wr.w, _1, _2, _3));
    for (int test_num = 0; test_num < num_tests; test_num++)
    {
      float start_menc_angle   = g_menc_angle;
      float start_jenc_angle_0 = g_jenc_angle_0;
      float start_jenc_angle_1 = g_jenc_angle_1;
      wr.w->setUSBPower(g_chain, g_node, 7); // turn on peripheral ports 0-2
      ros::Time t_start(ros::Time::now());
      while (!wr.w->done_ && (ros::Time::now() - t_start).toSec() < 3.0)
        wr.w->listen(0.001);
      if (g_jenc_angle_0 != start_jenc_angle_0 &&
          g_jenc_angle_1 != start_jenc_angle_1 &&
          g_menc_angle   != start_menc_angle)
        printf("test %d OK   (%.1f %.1f %.1f)\n", 
               test_num, 
               g_menc_angle, g_jenc_angle_0, g_jenc_angle_1);
      else
        printf("test %d FAIL (%.1f %.1f %.1f)\n", 
               test_num,
               g_menc_angle, g_jenc_angle_0, g_jenc_angle_1);
      wr.w->setUSBPower(g_chain, g_node, 0);
      t_start = ros::Time::now();
      while (!wr.w->done_ && (ros::Time::now() - t_start).toSec() < 2.0)
        wr.w->listen(0.001);
    }
  }
  else
    printf("unknown command: [%s]\n", cmd);
  return 0;
}
