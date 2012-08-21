#pragma once

#include "ros/ros.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <cstdint>
#include <queue>

#include "../macros.h"

#include "../util/command.h"
#include "../util/vector2.h"
#include "../util/pose.h"

namespace se306p1 {
class Robot {
 private:
  ros::NodeHandle nh_;
  ros::Publisher doPublisher_;
  ros::Publisher goPublisher_;
  ros::Publisher scanPublisher_;

  void EnqueueCommand(Command c);

 public:
  Robot(uint64_t n);
  virtual ~Robot();

  enum class Readiness {
    NOT_READY,
    HALF_READY,
    READY
  };

  uint64_t id_;
  bool executing_;
  Pose pose_;
  Readiness readiness_;
  std::deque<Command> commands_;

  /**
   * Request the robot to go to a position via a child controller.
   */
  void Go(const Pose &pos, bool enqueue);

  void DispatchCommand();

  /**
   * Request the robot to stop immediately.
   */
  void Stop();

  /**
   * Request the robot repeatedly does something.
   */
  void Do(double lv, double av, bool enqueue);

  void ScanBarcode(int time, bool enqueue);
};
}
