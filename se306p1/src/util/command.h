#pragma once

#include <se306p1/Do.h>
#include <se306p1/Go.h>

#include <limits>

namespace se306p1 {
class Command {
 public:
  bool enqueue;
  bool isDo;
  double lv;
  double av;
  double x;
  double y;
  double theta;

  Command() {
    /**
     * Set the default values to 0 and make the command a Go command
     */
    this->enqueue = false;
    this->isDo = false;
    this->x = std::numeric_limits<double>::quiet_NaN();
    this->y = std::numeric_limits<double>::quiet_NaN();
    this->theta = std::numeric_limits<double>::quiet_NaN();
    this->lv = std::numeric_limits<double>::quiet_NaN();
    this->av = std::numeric_limits<double>::quiet_NaN();
  }
  Command(Do msg) {
    this->x = std::numeric_limits<double>::quiet_NaN();
    this->y = std::numeric_limits<double>::quiet_NaN();
    this->theta = std::numeric_limits<double>::quiet_NaN();
    /**
     * Constructor for Do commands
     */
    this->lv = msg.lv;
    this->av = msg.av;
    /**
     * isDo signifies if this->command is a Do or a Go.
     */
    this->isDo = true;
    this->enqueue = msg.enqueue;
  }

  Command(Go msg) {
    /**
     * Constructor for Go commands
     */
    this->x = msg.x;
    this->y = msg.y;
    this->theta = msg.theta;
    this->lv = std::numeric_limits<double>::quiet_NaN();
    this->av = std::numeric_limits<double>::quiet_NaN();
    this->isDo = false;
    this->enqueue = msg.enqueue;
  }

  virtual ~Command() {
    /**
     * Deconstrucot
     */
  }
};
}
