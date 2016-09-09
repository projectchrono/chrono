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
// HMMWV wheel subsystem
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_RigidTire::m_radius = 0.4673;
const double HMMWV_RigidTire::m_width = 0.254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_RigidTire::HMMWV_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    SetContactMaterial(0.9f, 0.1f, 2e7f, 0.3f);
    if (use_mesh) {
        SetMeshFilename(GetDataFile("hmmwv/hmmwv_tire.obj"), 0.005);
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
