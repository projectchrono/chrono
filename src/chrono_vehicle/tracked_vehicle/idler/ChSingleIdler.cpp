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
// Base class for a single idler (template definition).
// A single idler is of type LATERAL_PIN.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChSingleIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChSingleIdler::ChSingleIdler(const std::string& name) : ChIdler(name) {
}

// -----------------------------------------------------------------------------
void ChSingleIdler::Initialize(std::shared_ptr<ChChassis> chassis,
                               const ChVector<>& location,
                               ChTrackAssembly* track) {
    // Invoke the base class method
    ChIdler::Initialize(chassis, location, track);

    CreateContactMaterial(m_wheel->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == m_wheel->GetSystem()->GetContactMethod());

    // Add contact geometry
    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetFamily(TrackedCollisionFamily::IDLERS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::WHEELS);

    if (track->IsIdlerCylinder()) {
        m_wheel->GetCollisionModel()->AddCylinder(m_material, radius, radius, width / 2);
    } else {
        m_wheel->GetCollisionModel()->AddCylindricalShell(m_material, radius, width / 2);
    }

    m_wheel->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
void ChSingleIdler::AddVisualizationAssets(VisualizationType vis) {
    ChIdler::AddVisualizationAssets(vis);

    if (vis != VisualizationType::PRIMITIVES)
        return;

    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    cyl->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl);

    auto tex = chrono_types::make_shared<ChTexture>();
    tex->SetTextureFilename(chrono::GetChronoDataFile("textures/bluewhite.png"));
    m_wheel->AddAsset(tex);
}

void ChSingleIdler::RemoveVisualizationAssets() {
    ChIdler::RemoveVisualizationAssets();

    m_wheel->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
