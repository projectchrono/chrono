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
// Base class for segmented track assemblies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"

namespace chrono {
namespace vehicle {

ChTrackAssemblySegmented::ChTrackAssemblySegmented(const std::string& name, VehicleSide side)
    : ChTrackAssembly(name, side), m_connection_type(ConnectionType::IDEAL_JOINT) {}

}  // end namespace vehicle
}  // end namespace chrono
