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

void FSM::determineNextState(int &egoLaneId, vector<bool> &laneFree, vector<double> &laneSpeed_m_s,
                             double max_speed_m_s) {
    int right = egoLaneId + 1;
    int left = egoLaneId - 1;
    const bool rightPossible = right < laneFree.size() && laneFree[right];
    const bool leftPossible = left >= 0 && laneFree[left];

    const bool egoFastest = laneSpeed_m_s[egoLaneId] == max_speed_m_s;
    const bool rightFasterThanLeft = laneSpeed_m_s[right] > laneSpeed_m_s[left];
    const bool leftFasterThanEgo = laneSpeed_m_s[left] > laneSpeed_m_s[egoLaneId];
    const bool rightFasterThanEgo = laneSpeed_m_s[right] > laneSpeed_m_s[egoLaneId];

    if (egoFastest) {
        this->keepLane();
    } else if (rightFasterThanLeft && rightPossible) {
        this->changeLaneRight(egoLaneId);
    } else if (leftFasterThanEgo && leftPossible) {
        this->changeLaneLeft(egoLaneId);
    } else if (rightFasterThanEgo && rightPossible) {
        this->changeLaneRight(egoLaneId);
    }
}
