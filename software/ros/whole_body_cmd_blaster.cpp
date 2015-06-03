#include <ros/ros.h>
#include "wandrr_controller.h"
#include "std_msgs/Float32.h"
#include <sched.h>
using namespace wandrr;

static StepprMegaCmd g_mc;

static float g_target_angle = 0;
static float g_kp = -1;

void state_cb(WandrrController *wc, const StepprMegaState * const state)
{
  ros::Time t(ros::Time::now());
#if 0
  if ((t - t_last_print).toSec() > 0.1)
  {
    t_last_print = t;
    printf("target = %.3f angle = %.3f  effort = %.3f\n",
           target, angle, effort);
    /*
    for (int j = 0; j < 15; j++)
    {
      for (int i = 0; i < 7; i++)
      {
        const int ID_PRES_0 = StepprInterface::SENSOR_SLOT_ID_PRESSURE_0;
        const int ID_PRES_3 = StepprInterface::SENSOR_SLOT_ID_PRESSURE_3;
        if (state->joints[j].sensor_slot_ids[i] >= ID_PRES_0 &&
            state->joints[j].sensor_slot_ids[i] <= ID_PRES_3 )
        {
          printf("   joint %d id %d val = %6d\n",
                 j,
                 state->joints[j].sensor_slot_ids[i],
                 state->joints[j].sensor_slots[i]);
        }
      }
    }
    */
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "whole_body_cmd_blaster");
  ros::NodeHandle nh, nh_private = ros::NodeHandle("~");
  memset(&g_mc, 0, sizeof(StepprMegaCmd));
  WandrrController wc("eth0");
  wc.setStateCallback(state_cb);
  ROS_INFO("whole body command blaster init complete\n");

  sched_param param;
  param.sched_priority = 45;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
  {
    perror("sched_setscheduler failed");
    return 1;
  }

  g_mc.joint_cmds[0].control_id = 0x42;
  g_mc.joint_cmds[0].control_mode = 3;
  g_mc.joint_cmds[0].targets[0] = 0.123;
  g_mc.joint_cmds[0].targets[1] = 0;

  while (ros::ok())
  {
    ros::spinOnce();
    wc.listen(0.001);
    wc.txMegaCmd(&g_mc);
  }
  return 0;
}
