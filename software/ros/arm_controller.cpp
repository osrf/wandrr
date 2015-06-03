#include "mini_state.h"
#include "local_mcast_rx.h"
#include "serial_router.h"
#include <ros/ros.h>
#include <termios.h>
#include <string>
using namespace wandrr;
using std::string;

/////////////////////////////////////////////////////////////////////////
// CONTROLLERS
/////////////////////////////////////////////////////////////////////////

class ArmController
{
public:
  float goals[12];
  virtual void tick(MiniState *ms) { }
  string name_, desc_;
  ArmController(string name, string desc)
  : name_(name), desc_(desc)
  { 
    for (int i = 0; i < 12; i++) 
      goals[i] = 0; 
  }
  const static int L_SHOULDER_Y = 0;
  const static int L_SHOULDER_X = 1;
  const static int L_SHOULDER_Z = 2;
  const static int L_ELBOW_Y    = 3;
  const static int L_FOREARM_Z  = 4;
  const static int R_SHOULDER_Y = 5;
  const static int R_SHOULDER_X = 6;
  const static int R_SHOULDER_Z = 7;
  const static int R_ELBOW_Y    = 8;
  const static int R_FOREARM_Z  = 9;
  const static int N_PAN_Z      =10;
  const static int N_TILT_Y     =11;
  const static float ENC_2_RAD  = 6.28318/16384.0;
};

class Zombie : public ArmController
{
public:
  Zombie() : ArmController("zombie", "arms straight forwards") 
  { 
    goals[L_SHOULDER_Y] =  0   ;
    goals[L_SHOULDER_X] =  1.4 ;
    goals[L_SHOULDER_Z] =  0.2 ;
    goals[L_ELBOW_Y   ] =  0   ;
    goals[R_SHOULDER_Y] =  0   ;
    goals[R_SHOULDER_X] = -1.4 ;
    goals[R_SHOULDER_Z] =  0   ;
    goals[R_ELBOW_Y   ] =  0   ;
  }
};

class Down : public ArmController
{ 
public:
  Down() : ArmController("down", "arms pointed straight down")
  { 
    goals[L_SHOULDER_Y] =  1.57;
    goals[L_SHOULDER_X] =  1.4 ;
    goals[L_SHOULDER_Z] =  0.2 ;
    goals[L_ELBOW_Y   ] =  0   ;
    goals[R_SHOULDER_Y] = -1.57;
    goals[R_SHOULDER_X] = -1.4 ;
    goals[R_SHOULDER_Z] =  0   ;
    goals[R_ELBOW_Y   ] =  0   ;
  }
};

class Bent : public ArmController
{
public:
  Bent() : ArmController("bent", "90 degree elbow angles")
  {
    goals[L_SHOULDER_Y] =  1.57;
    goals[L_SHOULDER_X] =  1.57;
    goals[L_SHOULDER_Z] =  0.2 ;
    goals[L_ELBOW_Y   ] =  1.57;
    goals[R_SHOULDER_Y] = -1.57;
    goals[R_SHOULDER_X] = -1.57;
    goals[R_SHOULDER_Z] =  0   ;
    goals[R_ELBOW_Y   ] = -1.57;
  }
};

class Touchdown : public ArmController
{
public:
  Touchdown() : ArmController("touchdown", "6 points")
  {
    goals[L_SHOULDER_Y] =  0   ;
    goals[L_SHOULDER_X] =  0   ;
    goals[L_SHOULDER_Z] =  0.2 ;
    goals[L_ELBOW_Y   ] =  1.57;
    goals[R_SHOULDER_Y] =  0   ;
    goals[R_SHOULDER_X] =  0   ;
    goals[R_SHOULDER_Z] =  0   ;
    goals[R_ELBOW_Y   ] = -1.57;
  }
};

class Pump : public ArmController
{
public:
  Pump() : ArmController("pump", "athletic motions while walking")
  {
    goals[L_SHOULDER_Y] =  1.57;
    goals[L_SHOULDER_X] =  1.4 ;
    goals[L_SHOULDER_Z] =  0.2 ;
    goals[L_ELBOW_Y   ] =  1.57;
    goals[R_SHOULDER_Y] = -1.57;
    goals[R_SHOULDER_X] = -1.4 ;
    goals[R_SHOULDER_Z] =  0   ;
    goals[R_ELBOW_Y   ] = -1.57;
  }
  virtual void tick(MiniState *ms)
  {
    // i'm sure these are all wrong
    goals[L_SHOULDER_Y] =  1.3 + ENC_2_RAD * ms->joint_angles[1] - 5.1; // left shoulder Y = right hip Y
    goals[R_SHOULDER_Y] = -1.3 + ENC_2_RAD * ms->joint_angles[7] - 4.1; // right shoulder Y = left hip Y
  }
};

/////////////////////////////////////////////////////////////////////////
// STATE CALLBACK
/////////////////////////////////////////////////////////////////////////

MiniState g_ms;

void mini_state_cb(const uint8_t * const data, const uint16_t len)
{
  if (sizeof(g_ms) == len)
    g_ms = *((MiniState *)data);
}

/////////////////////////////////////////////////////////////////////////
// ENORMOUS ALL-POWERFUL MAIN FUNCTION, YES I KNOW THIS IS TERRIBLE
/////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_controller");
  if (argc < 2)
  {
    ROS_FATAL("syntax: arm_controller RS485_ROUTER_NETWOK_INTERFACE");
    return 1;
  }
  ros::NodeHandle nh_private("~");
  const char *iface = argv[1];
  SerialRouter sr(iface);
  for (int i = 0; i < 3; i++)
    sr.setPortPower(i, true);
  usleep(100000); // let dynamixels boot
  const static int NUM_ARM_JOINTS = 5, NUM_NECK_JOINTS = 2;

  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    sr.setDmxlLED(0, 1+i, true);
    usleep(100);
    sr.setDmxlEnable(0, i+1, true);
    usleep(100);
    usleep(100);
    sr.setDmxlLED(1, 1+i, true);
    usleep(100);
    sr.setDmxlEnable(1, i+1, true);
    usleep(100);
  }
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
    sr.setDmxlLED(2, i+1, true);
    usleep(100);
    sr.setDmxlEnable(2, i+1, true);
    usleep(100);
  }

  LocalMCastRx lmcr("224.0.0.149", 11399);
  lmcr.setCallback(mini_state_cb);
  // set up raw keyboard
  termios old_attrs, new_attrs;
  tcgetattr(fileno(stdin), &old_attrs);
  memcpy(&new_attrs, &old_attrs, sizeof(new_attrs));
  new_attrs.c_lflag &= ~(ECHO | ICANON);
  new_attrs.c_cc[VTIME] = 0;
  new_attrs.c_cc[VMIN] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_attrs);

  std::vector<ArmController *> arm_controllers;
  arm_controllers.push_back(new Down());
  arm_controllers.push_back(new Zombie());
  arm_controllers.push_back(new Bent());
  arm_controllers.push_back(new Touchdown());
  arm_controllers.push_back(new Pump());

  printf("===============================================================\n");
  printf("Press the first letter to switch between available controllers:\n");
  printf("when done, press 'q' to quit.\n");
  printf("===============================================================\n");
  for (size_t i = 0; i < arm_controllers.size(); i++)
  {
    ArmController *ac = arm_controllers[i];
    printf("%c: %s = %s\n", 
           ac->name_[0], ac->name_.c_str(), ac->desc_.c_str());
  }
  printf("===============================================================\n");
  printf("q: quit...\n");
  printf("===============================================================\n");
  ArmController *active_controller = arm_controllers[0]; // the down controller
  double controller_switch_time = 0;
  float switch_start_goals[12] = {0};
  float goals[12] = {0};
  int print_count = 0;
  while (ros::ok())
  {
    double t = ros::Time::now().toSec();
    lmcr.listen(0.01);
    int c = fgetc(stdin);
    if (c == 'q')
      break;
    if (c > 0)
    {
      for (size_t i = 0; i < arm_controllers.size(); i++)
        if ((char)c == arm_controllers[i]->name_[0])
        {
          if (active_controller)
            for (int j = 0; j < 12; j++)
              switch_start_goals[j] = active_controller->goals[j];
          active_controller = arm_controllers[i];
          controller_switch_time = t;
        }
    }
    if (active_controller)
    {
      active_controller->tick(&g_ms);
      float *pgoals = active_controller->goals;
      // move towards the goal at maximum ramp rates
      const double running_time = t - controller_switch_time;
      const double SWITCH_TIME = 2.0;
      if (running_time < SWITCH_TIME)
      {
        // gradually slide towards the new controller's goal
        double x = running_time / SWITCH_TIME;
        for (int i = 0; i < 12; i++)
          goals[i] = (1.0 - x) * switch_start_goals[i] + 
                            x  * active_controller->goals[i];
        pgoals = goals;
      }
      sr.setWandrrDmxlGoals(pgoals);
      if (++print_count % 10 == 0) // print at 10 Hz
      {
        printf("\r");
        for (int i = 0; i < 12; i++)
        {
          printf("%5.2f ", pgoals[i]);
          if (i == 4 || i == 9)
            printf("    ");
        }
      }
    }
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
  tcsetattr(fileno(stdin), TCSANOW, &old_attrs);
  printf("goodbye. have a nice day.\n");
  return 0;
}

