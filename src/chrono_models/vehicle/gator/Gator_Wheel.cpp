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
// Gator wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/gator/Gator_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_Wheel::m_mass = 11.0;
const ChVector<> Gator_Wheel::m_inertia(0.24, 0.42, 0.24);

const double Gator_Wheel::m_radius = 0.3365;
const double Gator_Wheel::m_width = 0.205;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_Wheel::Gator_Wheel(const std::string& name) : ChWheel(name) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
