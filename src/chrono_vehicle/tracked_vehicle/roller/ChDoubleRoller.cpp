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
// Base class for a double roller (template definition).
// A double roller is of type CENTRAL_PIN.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/roller/ChDoubleRoller.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleRoller::ChDoubleRoller(const std::string& name) : ChRoller(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoller::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class method
    ChRoller::Initialize(chassis, location);

    // Add contact geometry.
    double radius = GetRadius();
    double width = 0.5 * (GetWidth() - GetGap());
    double offset = 0.25 * (GetWidth() + GetGap());

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetFamily(TrackedCollisionFamily::ROLLERS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::WHEELS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::IDLERS);

    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, offset, 0));
    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, -offset, 0));
    
    m_wheel->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleRoller::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double radius = GetRadius();
    double width = GetWidth();
    double gap = GetGap();

    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, gap / 2, 0);
    cyl_1->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -width / 2, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, -gap / 2, 0);
    cyl_2->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl_2);

    auto tex = std::make_shared<ChTexture>();
    tex->SetTextureFilename(chrono::GetChronoDataFile("redwhite.png"));
    m_wheel->AddAsset(tex);
}

void ChDoubleRoller::RemoveVisualizationAssets() {
    m_wheel->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
