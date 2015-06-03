#include <ros/ros.h>
#include "wandrr_controller.h"
#include "std_msgs/Float32.h"
#include <sched.h>
using namespace wandrr;

static ros::Time t_powerup;
static ros::Time t_last_print;
static StepprMegaCmd g_mc;

static float g_target_angle = 0;
static float g_kp = -1;

void target_angle_cb(const std_msgs::Float32::ConstPtr &msg)
{
  g_target_angle = msg->data;
}

void kp_cb(const std_msgs::Float32::ConstPtr &msg)
{
  g_kp = msg->data;
}

void state_cb(WandrrController *wc, const StepprMegaState * const state)
{
  ros::Time t(ros::Time::now());
  if (t_powerup.isZero())
    t_powerup = t;
  //ROS_INFO("state rx");
  static FILE *f = NULL;
  if (!f)
    f = fopen("times.txt", "w");
  fprintf(f, "%.6f\n", (t - t_powerup).toSec());

  const float target = g_target_angle; //6.28; //6.28; //0.0f;
  const float angle = state->joints[0].menc / 16384.0f * 2 * 3.14159;
  const float error = target - angle;
  const float kp = g_kp; //-5.0f;
  float effort = error * kp;
  const float max_effort = 10.0f;
  if (effort > max_effort)
    effort = max_effort;
  else if (effort < -max_effort)
    effort = -max_effort;

  g_mc.joint_cmds[0].control_id = 0x42;
  g_mc.joint_cmds[0].control_mode = (kp < -78 ? 0 : 3);
  g_mc.joint_cmds[0].targets[0] = effort;
  g_mc.joint_cmds[0].targets[1] = 0;

  g_mc.joint_cmds[1].control_id = 0x42;
  g_mc.joint_cmds[1].control_mode = (kp < -78 ? 0 : 3);
  g_mc.joint_cmds[1].targets[0] = effort;
  g_mc.joint_cmds[1].targets[1] = 0;

  wc->txMegaCmd(&g_mc);

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
  ros::init(argc, argv, "whole_body_control");
  ros::NodeHandle nh, nh_private = ros::NodeHandle("~");
  ros::Subscriber target_angle_sub = nh.subscribe<std_msgs::Float32>
    ("target_angle", 1, target_angle_cb);
  ros::Subscriber kp_sub = nh.subscribe<std_msgs::Float32>("kp", 1, kp_cb);
  memset(&g_mc, 0, sizeof(StepprMegaCmd));
  WandrrController wc("eth0");
  wc.setStateCallback(state_cb);
  ROS_INFO("whole body controller init complete\n");

  sched_param param;
  param.sched_priority = 45;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
  {
    perror("sched_setscheduler failed");
    return 1;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    wc.listen(0.00001);
  }
  return 0;
}
