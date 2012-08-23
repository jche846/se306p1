#pragma once

#include "ros/ros.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <cstdint>
#include <queue>

#include "../util/command.h"
#include "../util/vector2.h"
#include "../util/pose.h"

namespace se306p1 {
/**
 * A representation of a robot in the supervisor.
 */
class Robot {
 private:
  ros::NodeHandle nh_; ///< ROS node handle.
  ros::Publisher doPublisher_; ///< Publisher for Do.
  ros::Publisher goPublisher_; ///< Publisher for Go.
  ros::Publisher scanPublisher_; ///< Publisher for Scan.

  /// Place a command on the queue.
  void EnqueueCommand(Command c);

 public:
  /**
   * Construct a robot using a given ID.
   */
  Robot(uint64_t n);
  virtual ~Robot();

  /**
   * The ready states of the robot.
   */
  enum class Readiness {
    NOT_READY,
    HALF_READY,
    READY
  };

  /// Robot identification.
  uint64_t id_;

  /// Whether or not the robot is executing a command.
  bool executing_;

  /// The position and orientation of the robot.
  Pose pose_;

  /// The ready state of the robot.
  Readiness readiness_;

  /// Commands to be sent on the next ticks.
  std::deque<Command> commands_;

  /**
   * Request the robot to go to a position via a child controller.
   */
  void Go(const Pose &pos, bool enqueue,
          double errDist=0.00001, double errTheta=0.00001);

  /**
   * Dispatch a command waiting on the command queue.
   */
  void DispatchCommand();

  /**
   * Request the robot to stop immediately.
   */
  void Stop();

  /**
   * Request the robot repeatedly does something.
   */
  void Do(double lv, double av, bool enqueue);

  /**
   * Request the robot scans a barcode.
   */
  void ScanBarcode(int time, bool enqueue);
};
}
