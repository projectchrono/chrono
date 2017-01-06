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

#include "chrono_distributed/cosimulation/ChCosimulationDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

ChCosimulationDistr::ChCosimulationDistr(ChSystemDistr *my_sys) {
	this->my_sys = my_sys;
}

ChCosimulationDistr::~ChCosimulationDistr() {
}

} /* namespace chrono */
