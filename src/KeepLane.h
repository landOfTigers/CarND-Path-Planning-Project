#ifndef PATH_PLANNING_OFF_H
#define PATH_PLANNING_OFF_H

#include "State.h"

class KeepLane : public State {
public:
    explicit KeepLane(int &intendedLaneId);

    ~KeepLane() = default;

    void changeLaneLeft(FSM *m, int &currentLaneId) override;

    void changeLaneRight(FSM *m, int &currentLaneId) override;
};

#endif //PATH_PLANNING_OFF_H
