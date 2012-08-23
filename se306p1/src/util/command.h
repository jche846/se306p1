#pragma once

#define unicorn union

#include <se306p1/Scan.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
/**
 * The type of the command, for tagging the union.
 */
enum class CommandType {
  NONE,
  DO,
  GO,
  SCAN
};

/**
 * A tagged union for representing the Scan, Do and Go commands.
 */
class Command {
 public:
  CommandType type; ///< Tag for the command.
  bool enqueue; ///< Whether or not this command is to be enqueued.

  /// Union for commands.
  unicorn {
    /// Do command.
    struct {
      double lv; ///< Linear velocity.
      double av; ///< Angular velocity.
    };

    /// Go command.
    struct {
      double x; ///< x coordinate.
      double y; ///< y coordinate.
      double theta; ///< Orientation.
      double errDist; ///< Error for distance.
      double errTheta; ///< Error for theta.
    };

    /// Scan command.
    struct {
      int duration; ///< Duration of the scan.
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
    this->errDist = msg.errDist;
    this->errTheta = msg.errTheta;
    this->enqueue = msg.enqueue;
  }

  /**
   * Constructor for Scan commands
   */
  Command(Scan msg) {
    this->type = CommandType::SCAN;
    this->duration = msg.duration;
    this->enqueue = msg.enqueue;
  }

  virtual ~Command() {
  }
};
}
