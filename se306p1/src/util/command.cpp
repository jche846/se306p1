/*
 * command.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "command.h"

namespace se306p1 {
  //change to make ws commit work
  Command::Command(Do msg) {
    // Constructor for Do commands 
    this->lv = msg.lv;
    this->av = msg.av;
    //signifies if this->command is a Do or a Go.
    this->isDo = true;
    this->enqueue = msg.enqueue;
  }

  Command::Command(Go msg) {
    //Constructor for Go commands
    this->x = msg.x;
    this->y = msg.y;
    this->theta = msg.theta;
    this->isDo = false;
    this->enqueue = msg.enqueue;
  }

  Command::~Command() {
    //pass
  }
} /* namespace se306p1 */