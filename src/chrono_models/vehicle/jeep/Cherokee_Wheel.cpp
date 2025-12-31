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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Class for modeling a wheel for Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_Wheel.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables

const double Cherokee_Wheel::m_mass = 11.38;
const ChVector3d Cherokee_Wheel::m_inertia(0.5334, 0.9708, 0.5334);

const double Cherokee_Wheel::m_radius = 0.1905;
const double Cherokee_Wheel::m_width = 0.1524;

// -----------------------------------------------------------------------------

Cherokee_Wheel::Cherokee_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "jeep/Cherokee_Wheel.obj";
}

void Cherokee_Wheel::Construct(std::shared_ptr<ChChassis> chassis,
                               std::shared_ptr<ChSpindle> spindle,
                               VehicleSide side,
                               double offset) {
    ChWheel::Construct(chassis, spindle, side, offset);

    ChContactMaterialData mat_info;
    auto material = mat_info.CreateMaterial(spindle->GetSystem()->GetContactMethod());
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, m_radius, m_width);
    spindle->AddCollisionShape(ct_shape, ChFrame<>(ChVector3d(0, 0, m_offset), QuatFromAngleX(CH_PI_2)));
}

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
