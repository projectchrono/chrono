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

#include "chrono_distributed/collision/ChDataManagerDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"

namespace chrono {

ChDataManagerDistr::ChDataManagerDistr(ChSystemDistr *my_sys) {
	this->my_sys = my_sys;

	// The bodies lists are initialized to null
	local_bodylist = new ChBodyDistr* [my_sys->GetMaxLocal()]();
	shared_bodylist = new ChBodyDistr* [my_sys->GetMaxShared()]();
	ghost_bodylist = new ChBodyDistr* [my_sys->GetMaxGhost()]();
}

ChDataManagerDistr::~ChDataManagerDistr() {}

} /* namespace chrono */
