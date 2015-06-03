#ifndef WANDRR_MEGA_STATE_H
#define WANDRR_MEGA_STATE_H

#include <stdint.h>
#include "joint_state.h"
#include "serial_router.h"
#include "power_router.h"

namespace wandrr
{

class MegaState
{
public:
  static const int NUM_JOINTS = 15;
  JointState joints[NUM_JOINTS];
  SerialRouterState srs;
  PowerRouterState prs;
  MegaState();
};

}

#endif
