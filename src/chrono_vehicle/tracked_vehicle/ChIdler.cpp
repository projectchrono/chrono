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
// Base class for an idler subsystem.  An idler consists of the idler wheel and
// a connecting body.  The idler wheel is connected through a revolute joint to
// the connecting body which in turn is connected to the chassis through a
// translational joint.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

void ChIdler::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location, double pitch) {
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
