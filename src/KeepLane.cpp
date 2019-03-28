#include "KeepLane.h"
#include "ChangeLaneLeft.h"

KeepLane::KeepLane(int &intendedLaneId) {
    this->intendedLaneId = intendedLaneId;
}

void KeepLane::changeLaneLeft(FSM *m, int &currentLaneId) {
    m->setCurrent(new ChangeLaneLeft(currentLaneId));
    delete this;
}
