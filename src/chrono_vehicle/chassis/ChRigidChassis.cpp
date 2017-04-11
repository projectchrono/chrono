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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"

#include "chrono_vehicle/chassis/ChRigidChassis.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidChassis::ChRigidChassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = GetLocalPosCOM();
    m_body->AddAsset(sphere);
}

void ChRigidChassis::RemoveVisualizationAssets() {
    m_body->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
