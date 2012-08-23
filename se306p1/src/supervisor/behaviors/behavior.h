#pragma once

#include <memory>
#include <vector>

#include "../robot.h"

namespace se306p1 {
class Supervisor;

/**
 * A behavior represents something the supervisor can request its child robots
 * to perform.
 */
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
  Behavior(Supervisor &sup) : supervisor_(sup), done_(false) { };
  virtual ~Behavior() { };

  /**
   * Operations to perform when the behavior is ready.
   */
  virtual void Execute() = 0;

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

  /**
   * Whether or not the behavior has been executed.
   */
  bool done_;
};
}
