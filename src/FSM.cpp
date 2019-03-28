#include "FSM.h"
#include "KeepLane.h"

FSM::FSM() {
    int initialLaneId = 1;
    current = new KeepLane(initialLaneId);
}

void FSM::changeLaneLeft(int &currentLaneId) {
    current->changeLaneLeft(this, currentLaneId);
}

void FSM::changeLaneRight(int &currentLaneId) {
    current->changeLaneRight(this, currentLaneId);
}

void FSM::keepLane() {
    current->keepLane(this);
}

int FSM::getIntendedLaneId() {
    return current->getIntendedLaneId();
}
