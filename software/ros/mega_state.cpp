#include "mega_state.h"
#include <cstring>
using namespace wandrr;

MegaState::MegaState()
{
  for (int i = 0; i < NUM_JOINTS; i++)
    memset(&joints[i], 0, sizeof(JointState));
}
