#ifndef PATH_PLANNING_FSM_H
#define PATH_PLANNING_FSM_H

#include <vector>

using std::vector;

class FSM {

private:
    class State *current;

public:
    FSM();

    void setCurrent(State *s) {
        current = s;
    }

    void changeLaneLeft(int &currentLaneId);

    void changeLaneRight(int &currentLaneId);

    void keepLane();

    int getIntendedLaneId();
};

#endif //PATH_PLANNING_FSM_H
