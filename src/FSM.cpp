#include <algorithm>
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

void FSM::determineNextState(int &egoLaneId, vector<bool> &laneFree, vector<double> &v_lane_m_s, double v_max_m_s) {
    int right = egoLaneId + 1;
    int left = egoLaneId - 1;
    const bool rightPossible = right < laneFree.size() && laneFree[right];
    const bool leftPossible = left >= 0 && laneFree[left];

    const bool egoAtMaxSpeed = v_lane_m_s[egoLaneId] == v_max_m_s;
    const bool leftFasterThanEgo = v_lane_m_s[left] > v_lane_m_s[egoLaneId];
    const bool rightFasterThanEgo = v_lane_m_s[right] > v_lane_m_s[egoLaneId];

    long fastestLane = std::max_element(v_lane_m_s.begin(), v_lane_m_s.end()) - v_lane_m_s.begin();

    bool fastestLaneRight = fastestLane - egoLaneId > 0;
    bool fastestLaneLeft = fastestLane - egoLaneId < 0;

    if (egoAtMaxSpeed) {
        this->keepLane();
    } else if (fastestLaneRight && rightPossible) {
        this->changeLaneRight(egoLaneId);
    } else if (fastestLaneLeft && leftPossible) {
        this->changeLaneLeft(egoLaneId);
    } else if (rightFasterThanEgo && rightPossible) {
        this->changeLaneRight(egoLaneId);
    } else if (leftFasterThanEgo && leftPossible) {
        this->changeLaneLeft(egoLaneId);
    }
}
