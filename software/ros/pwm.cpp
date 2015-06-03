#include "wandrr_ros.h"
#include <signal.h>
using namespace wandrr;

static FILE *f = NULL;

void joint_state_cb(int chain, int hop, const JointState * const js)
{
  static int jscb_cnt = 0;
  jscb_cnt++;
  if (jscb_cnt % 1000 == 0)
  {
    printf("js %d:%d cb t = %ld  menc time = %d menc_raw = %6d menc angle = %.3f  %6d %6d %6d  (%.3f, %.3f) -> (%.3f %.3f)\n", 
           chain, 
           hop, 
           js->fpga_time,
           js->menc_time,
           js->menc_raw,
           js->menc_angle,
           js->adc[0],
           js->adc[1],
           js->adc[2],
           js->foc_d, js->foc_q,
           js->effort_d, js->effort_q);
  }
  if (!f)
    f = fopen("log.txt", "w");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pwm");
  if (argc < 4)
  {
    ROS_FATAL("syntax: pwm A B C");
    return 1;
  }
  WandrrRos wr(argc, argv);
  double t_start = ros::Time::now().toSec();
  wr.w->setJointStateCallback(joint_state_cb);
  const int hop = 0;
  wr.w->setUSBPower(hop, 1);
  wr.w->setLED(hop, true);
  wr.w->setTXInterval(hop, 0); // full speed ahead
  wr.w->setPWMParameters(hop);
  wr.w->setPWMDuty(hop, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
  ros::Duration(0.001).sleep(); // let a few pwm cycles go by
  wr.w->setEnableFETs(hop, true);
  ros::Duration(0.001).sleep(); // let a few pwm cycles go by
  while (!wr.w->done)
  {
    wr.w->listen(0.001);
    ros::spinOnce();
  }
  ROS_INFO("exiting...");
  wr.w->setPWMDuty(hop, 2559, 2559, 2559); // move back to 50% duty cycles
  wr.w->setLED(hop, false);
  wr.w->setEnableFETs(hop, false);
  wr.w->setTXInterval(hop, 4883); // about 4 Hz
  usleep(10000);
  return 0;
}
