#include "wandrr_ros.h"
#include "yaml-cpp/yaml.h"
#include <boost/function.hpp>
#include <ros/ros.h>
#include "serial_router.h"
using namespace wandrr;

double linear_interpolate(double a, double b, double x)
{
  return (1.0 - x) * a + x * b;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "play_dmxl_script");
  if (argc < 3)
  {
    ROS_FATAL("syntax: play_dmxl_script NETWOK_INTERFACE SCRIPTFILE");
    return 1;
  }
  ros::NodeHandle nh_private("~");
  const char *iface = argv[1];
  SerialRouter sr(iface);
  for (int i = 0; i < 3; i++)
    sr.setPortPower(i, true);
  usleep(100000); // let dynamixels boot

  const static int NUM_ARM_JOINTS = 5, NUM_NECK_JOINTS = 2;
//#define SET_MAX_TORQUE
#ifdef SET_MAX_TORQUE
  const static int MAX_TORQUE = 850; //0x3ff; // half torque = max
#endif
  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    sr.setDmxlLED(0, 1+i, true);
    usleep(100);
    sr.setDmxlEnable(0, i+1, true);
    usleep(100);
#ifdef SET_MAX_TORQUE
    sr.setDmxlMaxTorque(0, i+1, MAX_TORQUE);
#endif
    usleep(100);
    sr.setDmxlLED(1, 1+i, true);
    usleep(100);
    sr.setDmxlEnable(1, i+1, true);
    usleep(100);
#ifdef SET_MAX_TORQUE
    sr.setDmxlMaxTorque(1, i+1, MAX_TORQUE);
#endif
    usleep(100);
  }
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
    sr.setDmxlLED(2, i+1, true);
    usleep(100);
    sr.setDmxlEnable(2, i+1, true);
    usleep(100);
#ifdef SET_MAX_TORQUE
    sr.setDmxlMaxTorque(2, i+1, MAX_TORQUE);
#endif
    usleep(100);
  }

  YAML::Node script = YAML::LoadFile(argv[2]);
  //double t_start = ros::Time::now().toSec();
  double left[NUM_ARM_JOINTS] = {0}, left_next[NUM_ARM_JOINTS] = {0};
  double right[NUM_ARM_JOINTS] = {0}, right_next[NUM_ARM_JOINTS] = {0};
  double neck[NUM_NECK_JOINTS] = {0}, neck_next[NUM_NECK_JOINTS] = {0};
  double left_interp[NUM_ARM_JOINTS] = {0};
  double right_interp[NUM_ARM_JOINTS] = {0};
  double neck_interp[NUM_NECK_JOINTS] = {0};
  ros::Time t_start(ros::Time::now());
  double t_elapsed = 0;
  double t_prev_tx = 0;
  float mega_goals[12] = {0};
  for (size_t i = 0; i < script.size(); i++)
  {
    const double dt_next = script[i]["dt"].as<double>();
    for (int j = 0; j < NUM_ARM_JOINTS; j++)
    {
      left_next[j]  = script[i]["left"][j].as<double>();
      right_next[j] = script[i]["right"][j].as<double>();
      if (i == 0)
      {
        left[j] = left_next[j];
        right[j] = right_next[j];
      }
    }
    for (int j = 0; j < NUM_NECK_JOINTS; j++)
    {
      neck_next[j]  = script[i]["neck"][j].as<double>();
      if (i == 0)
        neck[j] = neck_next[j];
    }
    double t_next = t_elapsed + dt_next;
    double t_waypoint_start = t_elapsed;
    
    /*
    while (t_elapsed < t_next && ros::ok())
    {
      t_elapsed = (ros::Time::now() - t_start).toSec();
      usleep(1000);
    }
    */
    printf("\n");
    printf("%.2f-second motion to:\n", dt_next);
    printf("left:  %6.2f %6.2f %6.2f %6.2f %6.2f\n", 
           left_next[0], left_next[1], left_next[2], 
           left_next[3], left_next[4]);
    printf("right: %6.2f %6.2f %6.2f %6.2f %6.2f\n", 
           right_next[0], right_next[1], right_next[2], 
           right_next[3], right_next[4]);
    printf("neck:  %6.2f %6.2f\n",
           neck_next[0], neck_next[1]);
    
    while (t_elapsed < t_next && ros::ok())
    {
      t_elapsed = (ros::Time::now() - t_start).toSec();
      ros::spinOnce();
      usleep(1000);

      if (t_elapsed - t_prev_tx > 0.01) // send interpolated targets at 100 hz
      {
        t_prev_tx = t_elapsed;
        double t_frac = (t_elapsed - t_waypoint_start) / dt_next;
        //printf("                 t_frac = %.4f\n", t_frac);
        for (int j = 0; j < NUM_ARM_JOINTS; j++)
        {
          left_interp[j] = linear_interpolate(left[j],  left_next[j],  t_frac);
          right_interp[j] = linear_interpolate(right[j], right_next[j], t_frac);
          // send left_interp and right_interp to servo J on right arm
          mega_goals[j] = left_interp[j];
          mega_goals[j+NUM_ARM_JOINTS] = right_interp[j];
          //sr.setDmxlGoalRadians(0, j+1, left_interp[j]);
          //sr.setDmxlGoalRadians(1, j+1, right_interp[j]);
        }
        for (int j = 0; j < NUM_NECK_JOINTS; j++)
        {
          neck_interp[j] = linear_interpolate(neck[j],  neck_next[j],  t_frac);
          // send neck_interp to servo J on neck
          mega_goals[j+2*NUM_ARM_JOINTS] = neck_interp[j];
          //sr.setDmxlGoalRadians(2, j+1, neck_interp[j]);
        }
        sr.setWandrrDmxlGoals(mega_goals);
        /*
        printf("\n");
        printf("left:  %6.1f %6.1f %6.1f %6.1f %6.1f\n", 
               left_interp[0], left_interp[1], left_interp[2], left_interp[3], left_interp[4]);
        printf("right: %6.1f %6.1f %6.1f %6.1f %6.1f\n", 
               right_interp[0], right_interp[1], right_interp[2], right_interp[3], right_interp[4]);
        printf("neck:  %6.1f %6.1f\n",
               neck_interp[0], neck_interp[1]);
        */
      }
    }
    
    t_elapsed = (ros::Time::now() - t_start).toSec();
    t_prev_tx = t_elapsed;
    for (int j = 0; j < NUM_ARM_JOINTS; j++)
    {
      left[j] = left_next[j];
      right[j] = right_next[j];
      // send left[j] to servo J on left
      // send right[j] to servo J on right
      //sr.setDmxlGoalRadians(0, j+1, left[j]);
      //sr.setDmxlGoalRadians(1, j+1, right[j]);
      mega_goals[j] = left[j];
      mega_goals[j+NUM_ARM_JOINTS] = right[j];
    }
    for (int j = 0; j < NUM_NECK_JOINTS; j++)
    {
      neck[j] = neck_next[j];
      // send neck[j] to servo J on neck
      //sr.setDmxlGoalRadians(2, j+1, neck[j]);
      mega_goals[j+2*NUM_ARM_JOINTS] = neck[j];
    }
    sr.setWandrrDmxlGoals(mega_goals);

    //ROS_INFO("%.6f sending node %d = %.3f amps, damp = %.3f", 
    //         t_prev, h, current, damping);
    //wr.w->setFOCTarget(c, h, current); 
    //wr.w->setMotionCommand(c, h, 3, current, damping, control_id++);
  }
  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    sr.setDmxlLED(0, 1+i, false);
    sr.setDmxlEnable(0, i+1, false);
    sr.setDmxlLED(1, 1+i, false);
    sr.setDmxlEnable(1, i+1, false);
  }
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
    sr.setDmxlLED(2, i+1, false);
    sr.setDmxlEnable(2, i+1, false);
  }
  return 0;
}
