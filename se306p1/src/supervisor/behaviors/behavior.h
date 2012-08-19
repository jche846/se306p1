#pragma once

#include <memory>
#include <vector>

#include "../robot.h"

namespace se306p1 {
class Supervisor;

class Behavior {
protected:
  Supervisor &supervisor_;

  void MoveNodesToDests(const std::vector<std::shared_ptr<Robot> > &nodes,
                        const std::vector<Pose> &poses);

public:
  Behavior(Supervisor &sup);
  virtual ~Behavior();

  virtual void Initialize() = 0;
  virtual void Tick() = 0;

  typedef std::unique_ptr<Behavior> BehaviorFactory(Supervisor &sup);

  template<typename T>
  static std::unique_ptr<Behavior> construct(Supervisor &sup) {
    return std::unique_ptr<Behavior>(dynamic_cast<Behavior *>(new T(sup)));
  };
};
}
