#pragma once

#include <memory>
#include <vector>

#include "../robot.h"

namespace se306p1 {
class Supervisor;

class Behavior {
protected:
  Supervisor &supervisor_;

public:
  Behavior(Supervisor &sup) : supervisor_(sup) { };
  virtual ~Behavior() { };

  virtual void Tick() = 0;

  typedef std::unique_ptr<Behavior> BehaviorFactory(Supervisor &sup);

  template<typename T>
  static std::unique_ptr<Behavior> construct(Supervisor &sup) {
    return std::unique_ptr<Behavior>(dynamic_cast<Behavior *>(new T(sup)));
  };
};
}
