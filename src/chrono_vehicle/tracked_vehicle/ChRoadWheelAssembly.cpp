// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a road wheel assembly (suspension).  A road wheel assembly
// contains a road wheel body (connected through a revolute joint to the chassis)
// with different suspension topologies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRoadWheelAssembly::ChRoadWheelAssembly(const std::string& name) : ChPart(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRoadWheelAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    m_road_wheel->Initialize(chassis, GetCarrierBody(), location);
}

}  // end namespace vehicle
}  // end namespace chrono
