#include "wandrr_ros.h"
#include <signal.h>
using namespace wandrr;

static WandrrRos *g_wr = NULL;

void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    if (g_wr)
      g_wr->w->done_ = true;
}

WandrrRos::WandrrRos(int argc, char **argv)
: nh_private("~"), w(NULL)
{
  g_wr = this;
  nh_private = ros::NodeHandle("~");
  w = new Wandrr();
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  nh_private.param<bool>("log", w->logging_, false);
  ROS_INFO("WandrrRos init complete");
}

WandrrRos::~WandrrRos()
{
  delete w;
  w = NULL;
}

void WandrrRos::spin()
{
  for (int loop_count = 0; !w->done_; loop_count++)
  {
    ros::spinOnce();
    w->listen(0.001);
  }
  // todo: shutdown motors ?
}

