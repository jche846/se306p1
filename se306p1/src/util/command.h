#pragma once

#include <se306p1/Scan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>

#include <limits>

namespace se306p1 {
enum class CommandType {
  NONE,
  DO,
  GO,
  SCAN
};

class Command {
 public:
  bool enqueue;
  CommandType type;
  double lv;
  double av;
  double x;
  double y;
  double theta;

  /**
   * A null command.
   */
  Command() {
    this->enqueue = false;
    this->type = CommandType::NONE;
    this->x = std::numeric_limits<double>::quiet_NaN();
    this->y = std::numeric_limits<double>::quiet_NaN();
    this->theta = std::numeric_limits<double>::quiet_NaN();
    this->lv = std::numeric_limits<double>::quiet_NaN();
    this->av = std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * Constructor for Do commands
   */
  Command(Do msg) {
    this->x = std::numeric_limits<double>::quiet_NaN();
    this->y = std::numeric_limits<double>::quiet_NaN();
    this->theta = std::numeric_limits<double>::quiet_NaN();
    this->lv = msg.lv;
    this->av = msg.av;
    this->enqueue = msg.enqueue;

    // CommandType signifies if this->command is a Do or a Go.
    this->type = CommandType::DO;
  }

  /**
   * Constructor for Go commands
   */
  Command(Go msg) {
    this->x = msg.x;
    this->y = msg.y;
    this->theta = msg.theta;
    this->enqueue = msg.enqueue;

    this->lv = std::numeric_limits<double>::quiet_NaN();
    this->av = std::numeric_limits<double>::quiet_NaN();

    this->type = CommandType::GO;
  }

  Command(Scan msg) {
    this->type = CommandType::SCAN;
    this->enqueue = msg.enqueue;
    this->x = std::numeric_limits<double>::quiet_NaN();
    this->y = std::numeric_limits<double>::quiet_NaN();
    this->theta = std::numeric_limits<double>::quiet_NaN();
    this->lv = std::numeric_limits<double>::quiet_NaN();
    this->av = std::numeric_limits<double>::quiet_NaN();
  }

  virtual ~Command() {
  }
};
}
