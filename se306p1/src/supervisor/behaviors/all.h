#pragma once

// Adding a new behavior.
//
// 1. Add the include to the list below.
// 2. Call this->RegisterBehavior<T>(); in RegisterBehaviors.

#include "rotate_behavior.h"

namespace se306p1 {
class Supervisor;

void Supervisor::RegisterBehaviors() {
  this->RegisterBehavior<RotateBehavior>();
}
}
