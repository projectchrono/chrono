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
// Base class for a double track wheel (template definition).
// A double track wheel is of type CENTRAL_PIN.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/ChDoubleTrackWheel.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

ChDoubleTrackWheel::ChDoubleTrackWheel(const std::string& name) : ChTrackWheel(name) {}

void ChDoubleTrackWheel::Initialize(std::shared_ptr<ChChassis> chassis,
                                    std::shared_ptr<ChBody> carrier,
                                    const ChVector<>& location,
                                    ChTrackAssembly* track) {
    // Invoke the base class method
    ChTrackWheel::Initialize(chassis, carrier, location, track);

    CreateContactMaterial(m_wheel->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == m_wheel->GetSystem()->GetContactMethod());

    // Add contact geometry
    double radius = GetRadius();
    double width = 0.5 * (GetWidth() - GetGap());
    double offset = 0.25 * (GetWidth() + GetGap());

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    if (track->IsRoadwheelCylinder()) {
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2, ChVector<>(0, +offset, 0));
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2, ChVector<>(0, -offset, 0));
    } else {
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2, ChVector<>(0, +offset, 0));
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2, ChVector<>(0, -offset, 0));
    }

    m_wheel->GetCollisionModel()->BuildModel();
}

void ChDoubleTrackWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double radius = GetRadius();
    double width = GetWidth();
    double gap = GetGap();

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

void ChDoubleTrackWheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_wheel);
}

}  // end namespace vehicle
}  // end namespace chrono
