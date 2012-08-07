/*
 * command.h
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 * 
 */

#pragma once

#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
  class Command {
  //change to make ws commit work
    public:
      bool enqueue;
      bool isDo;
      double lv;
      double av;
      double x;
      double y;
      double theta;

    Command() {
      this->enqueue = false;
      this->isDo = false;
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
      this->lv = 0.0;
      this->av = 0.0;
    }

    //change to make ws commit work
    Command(Do msg) {
      // Constructor for Do commands 
      this->lv = msg.lv;
      this->av = msg.av;
      //signifies if this->command is a Do or a Go.
      this->isDo = true;
      this->enqueue = msg.enqueue;
    }

    Command(Go msg) {
      //Constructor for Go commands
      this->x = msg.x;
      this->y = msg.y;
      this->theta = msg.theta;
      this->isDo = false;
      this->enqueue = msg.enqueue;
    }

    virtual ~Command() {
      //pass
    }
  };
}