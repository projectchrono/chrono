// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/ChDataManagerDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/physics/ChBody.h"

#include <memory>

namespace chrono {

ChDataManagerDistr::ChDataManagerDistr(ChSystemDistr *my_sys) : ChParallelDataManager() {
	this->my_sys = my_sys;
}

ChDataManagerDistr::~ChDataManagerDistr() {}


} /* namespace chrono */
