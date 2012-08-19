#pragma once

// Adding a new behavior.
//
// 1. Add the include to the list below.
// 2. Call sup.RegisterBehavior<T>(); in RegisterBehaviors.

#include "rotate_behavior.h"

namespace se306p1 {
class Supervisor;

inline void RegisterBehaviors(Supervisor &sup) {
  sup.RegisterBehavior<RotateBehavior>();
}
}