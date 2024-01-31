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
// Generic wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_Wheel.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables

const double Generic_Wheel::m_mass = 18.0;
const ChVector<> Generic_Wheel::m_inertia(0.4634, 0.6243, 0.4634);

const double Generic_Wheel::m_radius = 0.268;
const double Generic_Wheel::m_width = 0.22;

// -----------------------------------------------------------------------------

Generic_Wheel::Generic_Wheel(const std::string& name) : ChWheel(name) {}

void Generic_Wheel::Initialize(std::shared_ptr<ChChassis> chassis,
                               std::shared_ptr<ChBody> spindle,
                               VehicleSide side,
                               double offset) {
    ChWheel::Initialize(chassis, spindle, side, offset);

    ChContactMaterialData mat_info;
    auto material = mat_info.CreateMaterial(spindle->GetSystem()->GetContactMethod());
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, m_radius, m_width);
    spindle->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0, 0, m_offset), Q_from_AngX(CH_C_PI_2)));
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
