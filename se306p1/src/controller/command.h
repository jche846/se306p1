/*
 * command.h
 *
 *  Created on: Aug 4, 2012
 *      Author: ahug048
 */

#pragma once

#include <se306p1/Do.h>
#include <se306p1/Go.h>

namespace se306p1 {
  class Command {
  public:
    Command(Do msg);
    Command(Go msg);
    virtual ~Command();
  };
}
