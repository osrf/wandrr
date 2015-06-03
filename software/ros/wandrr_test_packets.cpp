#include "wandrr_ros.h"
#include "steppr_interface.h"
#include "serial_router.h"
using namespace wandrr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wandrr_test_packets");
  if (argc < 2)
  {
    ROS_FATAL("syntax: wandrr_test_packets PARAMFILE");
    return 1;
  }
  WandrrRos wr(argc, argv);
  if (!wr.w->initFromParamFile(argv[1]))
  {
    ROS_FATAL("couldn't load param file. adios amigo.");
    return 1;
  }

  OutboundPacket pkt;
  pkt.appendMotionCommand(0, 3, 1.23, 0.12, 0x12345678);
  wr.w->chains_[0]->tx(pkt);

  return 0;
}

