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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// (which are associated with the suspension's spindle body).
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

namespace chrono {
namespace vehicle {

ChWheel::ChWheel(const std::string& name) : ChPart(name) {}

// The base class initialization function attaches this wheel to the specified
// suspension spindle body (by incrementing the spindle's mass and inertia with
// that of the wheel.  A derived class should always invoke this base method.
void ChWheel::Initialize(std::shared_ptr<ChBody> spindle) {
    m_spindle = spindle;
    spindle->SetMass(spindle->GetMass() + GetMass());
    spindle->SetInertiaXX(spindle->GetInertiaXX() + GetInertia());
}

// -----------------------------------------------------------------------------
// Get the current COM location of the wheel subsystem.
// This is simply the COM of the associated spindle body.
// -----------------------------------------------------------------------------
ChVector<> ChWheel::GetCOMPos() const {
    return m_spindle->GetPos();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (GetRadius() == 0 || GetWidth() == 0)
        return;

    m_cyl_shape = std::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetWidth() / 2, 0);
    m_spindle->AddAsset(m_cyl_shape);
}

void ChWheel::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChWheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_cyl_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
