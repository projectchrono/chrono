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
// Base class for a single road wheel (template definition).
// A single road wheel is of type LATERAL_PIN.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChSingleRoadWheel.h"

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
                                   const ChVector<>& location) {
    // Invoke the base class method
    ChRoadWheel::Initialize(chassis, carrier, location);

    // Add contact geometry.
    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->SetFamily(TrackCollisionFamily::WHEELS);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackCollisionFamily::IDLERS);

    m_wheel->GetCollisionModel()->ClearModel();
    m_wheel->GetCollisionModel()->AddCylinder(radius, radius, width / 2);
    m_wheel->GetCollisionModel()->BuildModel();

    switch (m_wheel->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            m_wheel->GetMaterialSurface()->SetFriction(m_friction);
            m_wheel->GetMaterialSurface()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurfaceBase::DEM:
            m_wheel->GetMaterialSurfaceDEM()->SetFriction(m_friction);
            m_wheel->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
            m_wheel->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
            m_wheel->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
            break;
    }

    // Add visualization of the wheel.
    AddWheelVisualization();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleRoadWheel::AddWheelVisualization() {
    double radius = GetWheelRadius();
    double width = GetWheelWidth();

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    cyl->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl);

    auto tex = std::make_shared<ChTexture>();
    tex->SetTextureFilename(chrono::GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(tex);
}

}  // end namespace vehicle
}  // end namespace chrono
