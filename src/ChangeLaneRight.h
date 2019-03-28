#ifndef PATH_PLANNING_CHANGELANERIGHT_H
#define PATH_PLANNING_CHANGELANERIGHT_H

#include "State.h"

class ChangeLaneRight : public State {
public:

    explicit ChangeLaneRight(int &currentLaneId) {
        this->intendedLaneId = currentLaneId + 1;
    };

    ~ChangeLaneRight() = default;;

    void keepLane(FSM *m) override;
};

#endif //PATH_PLANNING_CHANGELANERIGHT_H
