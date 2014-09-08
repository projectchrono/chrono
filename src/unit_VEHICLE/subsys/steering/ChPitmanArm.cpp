// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// =============================================================================

#include "subsys/steering/ChPitmanArm.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPitmanArm::ChPitmanArm(const std::string& name)
: ChSteering(name)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPitmanArm::Initialize(ChSharedBodyPtr   chassis,
                             const ChVector<>& location)
{
}


}  // end namespace chrono
