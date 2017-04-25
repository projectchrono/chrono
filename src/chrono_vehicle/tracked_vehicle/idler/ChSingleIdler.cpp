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
// Base class for a single idler (template definition).
// A single idler is of type LATERAL_PIN.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChSingleIdler.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSingleIdler::ChSingleIdler(const std::string& name) : ChIdler(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleIdler::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class method
    ChIdler::Initialize(chassis, location);

    // Add contact geometry.
    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetFamily(TrackedCollisionFamily::IDLERS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::WHEELS);

    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2);

    m_wheel->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleIdler::AddVisualizationAssets(VisualizationType vis) {
    ChIdler::AddVisualizationAssets(vis);

    if (vis != VisualizationType::PRIMITIVES)
        return;

    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    cyl->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl);

    auto tex = std::make_shared<ChTexture>();
    tex->SetTextureFilename(chrono::GetChronoDataFile("bluwhite.png"));
    m_wheel->AddAsset(tex);
}

void ChSingleIdler::RemoveVisualizationAssets() {
    ChIdler::RemoveVisualizationAssets();

    m_wheel->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
