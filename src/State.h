#ifndef PATH_PLANNING_STATE_H
#define PATH_PLANNING_STATE_H

#include "FSM.h"

class State {

protected:
    int intendedLaneId = 1;

public:
    virtual void keepLane(FSM *m) {}

    virtual void changeLaneLeft(FSM *m, int &currentLaneId) {}

    virtual void changeLaneRight(FSM *m, int &currentLaneId) {}

    int getIntendedLaneId() {
        return intendedLaneId;
    }

};


#endif //PATH_PLANNING_STATE_H
