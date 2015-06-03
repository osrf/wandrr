#ifndef WANDRR_ROS_H
#define WANDRR_ROS_H

#include <ros/ros.h>
#include "wandrr.h"

namespace wandrr
{

class WandrrRos
{
public:
  ros::NodeHandle nh, nh_private;
  Wandrr *w;
  WandrrRos(int argc, char **argv);
  ~WandrrRos();
private:
  WandrrRos();
  WandrrRos(const WandrrRos &);
public:
  void spin();
};

}

#endif
