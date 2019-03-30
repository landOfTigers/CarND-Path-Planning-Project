#include "ChangeLaneLeft.h"
#include "KeepLane.h"

void ChangeLaneLeft::keepLane(FSM *m) {
  m->setCurrent(new KeepLane(this->intendedLaneId));
  delete this;
}
