#include "KeepLane.h"
#include "ChangeLaneLeft.h"
#include "ChangeLaneRight.h"

KeepLane::KeepLane(int &intendedLaneId) {
    this->intendedLaneId = intendedLaneId;
}

void KeepLane::changeLaneLeft(FSM *m, int &currentLaneId) {
    m->setCurrent(new ChangeLaneLeft(currentLaneId));
    delete this;
}

void KeepLane::changeLaneRight(FSM *m, int &currentLaneId) {
    m->setCurrent(new ChangeLaneRight(currentLaneId));
    delete this;
}

