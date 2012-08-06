/*
 * command.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#include "command.h"

namespace se306p1 {
  // get go data and check what type of command it is.
  Command::Command(Do msg) {
    // Constructor for do commands 
    
  }

  Command::Command(Go msg) {
    //Constructor for Go commands
  }

  Command::~Command() {
    // TODO Auto-generated destructor stub
  }
} /* namespace se306p1 */