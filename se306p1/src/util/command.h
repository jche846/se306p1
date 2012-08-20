#pragma once

#define unicorn union

#include <se306p1/Scan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
enum class CommandType {
  NONE,
  DO,
  GO,
  SCAN
};

class Command {
 public:
  CommandType type;
  bool enqueue;

  unicorn {
    // DO
    struct {
      double lv;
      double av;
    };

    // GO
    struct {
      double x;
      double y;
      double theta;
    };

    // SCAN
    struct {
      int duration;
    };
  };

  /**
   * A null command.
   */
  Command() {
    this->enqueue = false;
    this->type = CommandType::NONE;
  }

  /**
   * Constructor for Do commands
   */
  Command(Do msg) {
    this->type = CommandType::DO;
    this->lv = msg.lv;
    this->av = msg.av;
    this->enqueue = msg.enqueue;
  }

  /**
   * Constructor for Go commands
   */
  Command(Go msg) {
    this->type = CommandType::GO;
    this->x = msg.x;
    this->y = msg.y;
    this->theta = msg.theta;
    this->enqueue = msg.enqueue;
  }

  Command(Scan msg) {
    this->type = CommandType::SCAN;
    this->duration = msg.duration;
    this->enqueue = msg.enqueue;
  }

  virtual ~Command() {
  }
};
}
