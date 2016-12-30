/*
 * ChBodyDistr.cpp
 *
 *  Created on: Dec 29, 2016
 *      Author: nic
 */

#include "ChBodyDistr.h"
#include "chrono/physics/ChBody.h"

namespace chrono {

ChBodyDistr::ChBodyDistr()
: ChBody(),
  global_id(0)
{}

ChBodyDistr::~ChBodyDistr() {
}

} /* namespace chrono */
