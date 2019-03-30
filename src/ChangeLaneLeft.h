#ifndef PATH_PLANNING_ON_H
#define PATH_PLANNING_ON_H

#include "State.h"

class ChangeLaneLeft : public State {
 public:

  explicit ChangeLaneLeft(int &currentLaneId) {
    this->intendedLaneId = currentLaneId - 1;
  };

  ~ChangeLaneLeft() = default;;

  void keepLane(FSM *m) override;
};

#endif //PATH_PLANNING_ON_H
