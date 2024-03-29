#pragma once

// Adding a new behavior.
//
// 1. Add the include to the list below.
// 2. Call this->RegisterBehavior<T>(); in RegisterBehaviors.

#include "rotate_behavior.h"
#include "goto_circle_behavior.h"
#include "goto_triangle_behavior.h"
#include "goto_square_behavior.h"
#include "goto_pentagon_behavior.h"
#include "goto_hexagon_behavior.h"
#include "goto_heptagon_behavior.h"
#include "goto_octagon_behavior.h"

namespace se306p1 {
class Supervisor;

void Supervisor::RegisterBehaviors() {
  this->RegisterBehavior<RotateBehavior>();
  this->RegisterBehavior<GotoCircleBehavior>();
  this->RegisterBehavior<GotoTriangleBehavior>();
  this->RegisterBehavior<GotoSquareBehavior>();
  this->RegisterBehavior<GotoPentagonBehavior>();
  this->RegisterBehavior<GotoHexagonBehavior>();
  this->RegisterBehavior<GotoHeptagonBehavior>();
  this->RegisterBehavior<GotoOctagonBehavior>();
}
}
