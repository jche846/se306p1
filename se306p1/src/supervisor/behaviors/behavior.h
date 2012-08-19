#pragma once

#include <memory>
#include <vector>

#include "../robot.h"

namespace se306p1 {
class Supervisor;

class Behavior {
protected:
  /**
   * Supervisor attached to this behavior.
   */
  Supervisor &supervisor_;

public:
  /**
   * Construct a behavior attached to the given supervisor.
   */
  Behavior(Supervisor &sup) : supervisor_(sup) { };
  virtual ~Behavior() { };

  /**
   * Operations to perform on every tick of the ROS loop.
   */
  virtual void Tick() = 0;

  /**
   * Type which names a pointer to a specialization of Behavior::construct<T>.
   */
  typedef std::unique_ptr<Behavior> BehaviorFactory(Supervisor &sup);

  /**
   * Stand-in for a constructor pointer. Returns a pointer to a Behavior for
   * a given subclass of Behavior T.
   */
  template<typename T>
  static std::unique_ptr<Behavior> construct(Supervisor &sup) {
    return std::unique_ptr<Behavior>(dynamic_cast<Behavior *>(new T(sup)));
  };
};
}
