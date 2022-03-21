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
// Base class for a single road wheel (template definition).
// A single road wheel is of type LATERAL_PIN.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChSingleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSingleRoadWheel::ChSingleRoadWheel(const std::string& name) : ChRoadWheel(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleRoadWheel::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                   std::shared_ptr<ChBody> carrier,
                                   const ChVector<>& location,
                                   ChTrackAssembly* track) {
    // Invoke the base class method
    ChRoadWheel::Initialize(chassis, carrier, location, track);

    CreateContactMaterial(m_wheel->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == m_wheel->GetSystem()->GetContactMethod());

    // Add contact geometry
    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetFamily(TrackedCollisionFamily::WHEELS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::IDLERS);

    if (track->IsRoadwheelCylinder()) {
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2);
    } else {
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2);
    }

    m_wheel->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleRoadWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    cyl->GetCylinderGeometry().rad = radius;
    m_wheel->AddVisualShape(cyl);
}

void ChSingleRoadWheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_wheel);
}

}  // end namespace vehicle
}  // end namespace chrono
