#include "supervisor.h"

#include <se306p1/AskPosition.h>
#include <se306p1/Position.h>

namespace se306p1 {
  Supervisor::Supervisor() {
    // create publishers and subscribers
    _ansPosSubscriber = _nh.subscribe<Position>(ANS_POS_TOPIC, 1000, [this] (const boost::shared_ptr<const Position> msg) {
      this->ansPos_callback(*msg);
    });
    _askPosPublisher = _nh.advertise<AskPosition>(ASK_POS_TOPIC, 1000);
  }

  Supervisor::~Supervisor() { }

  void Supervisor::ansPos_callback(Position msg) {

  }

  void Supervisor::populate() {

  }

  void Supervisor::reset() {
    
  }

  void Supervisor::run() {
    this->populate();
  }
}