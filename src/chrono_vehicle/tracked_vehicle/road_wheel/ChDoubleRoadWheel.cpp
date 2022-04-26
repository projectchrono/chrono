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
// Base class for a double road wheel (template definition).
// A double road wheel is of type CENTRAL_PIN.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleRoadWheel::ChDoubleRoadWheel(const std::string& name) : ChRoadWheel(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoadWheel::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                   std::shared_ptr<ChBody> carrier,
                                   const ChVector<>& location,
                                   ChTrackAssembly* track) {
    // Invoke the base class method
    ChRoadWheel::Initialize(chassis, carrier, location, track);

    CreateContactMaterial(m_wheel->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == m_wheel->GetSystem()->GetContactMethod());

    // Add contact geometry
    double radius = GetWheelRadius();
    double width = 0.5 * (GetWheelWidth() - GetWheelGap());
    double offset = 0.25 * (GetWheelWidth() + GetWheelGap());

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetFamily(TrackedCollisionFamily::WHEELS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::IDLERS);

    if (track->IsRoadwheelCylinder()) {
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2, ChVector<>(0, +offset, 0));
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2, ChVector<>(0, -offset, 0));
    } else {
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2, ChVector<>(0, +offset, 0));
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2, ChVector<>(0, -offset, 0));
    }

    m_wheel->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoadWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double radius = GetWheelRadius();
    double width = GetWheelWidth();
    double gap = GetWheelGap();

    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, gap / 2, 0);
    cyl_1->GetCylinderGeometry().rad = radius;
    m_wheel->AddVisualShape(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -width / 2, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, -gap / 2, 0);
    cyl_2->GetCylinderGeometry().rad = radius;
    m_wheel->AddVisualShape(cyl_2);
}

void ChDoubleRoadWheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_wheel);
}

}  // end namespace vehicle
}  // end namespace chrono
