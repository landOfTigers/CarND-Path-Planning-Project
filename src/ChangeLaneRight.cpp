#include "ChangeLaneRight.h"
#include "KeepLane.h"

void ChangeLaneRight::keepLane(FSM *m) {
    m->setCurrent(new KeepLane(this->intendedLaneId));
    delete this;
}
