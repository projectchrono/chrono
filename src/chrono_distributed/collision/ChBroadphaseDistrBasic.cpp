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

#include "chrono_distributed/collision/ChBroadphaseDistrBasic.h"
#include "chrono_distributed/collision/ChBroadphaseDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

ChBroadphaseDistrBasic::ChBroadphaseDistrBasic(ChSystemDistr *my_sys) : ChBroadphaseDistr(my_sys)
{}

ChBroadphaseDistrBasic::~ChBroadphaseDistrBasic() {}

void ChBroadphaseDistrBasic::DetectPossibleCollisions()
{
	//TODO
}

} /* namespace chrono */
