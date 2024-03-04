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
#include "chrono/assets/ChVisualShapeCylinder.h"
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

    if (track->IsRoadwheelCylinder()) {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(m_material, radius, width);
        m_wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0, +offset, 0), Q_from_AngX(CH_C_PI_2)));
        m_wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0, -offset, 0), Q_from_AngX(CH_C_PI_2)));
    } else {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylindricalShell>(m_material, radius, width);
        m_wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0, +offset, 0), Q_from_AngX(CH_C_PI_2)));
        m_wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0, -offset, 0), Q_from_AngX(CH_C_PI_2)));
    }
}

void ChDoubleTrackWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double radius = GetRadius();
    double width = GetWidth();
    double gap = GetGap();

    ChVehicleGeometry::AddVisualizationCylinder(m_wheel,                      //
                                                ChVector<>(0, width / 2, 0),  //
                                                ChVector<>(0, gap / 2, 0),    //
                                                radius);

    ChVehicleGeometry::AddVisualizationCylinder(m_wheel,                       //
                                                ChVector<>(0, -width / 2, 0),  //
                                                ChVector<>(0, -gap / 2, 0),    //
                                                radius);
}

void ChDoubleTrackWheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_wheel);
}

}  // end namespace vehicle
}  // end namespace chrono
