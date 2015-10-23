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

#include "chrono_vehicle/tracked_vehicle/idler/ChSingleIdler.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSingleIdler::ChSingleIdler(const std::string& name) : ChIdler(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleIdler::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class method
    ChIdler::Initialize(chassis, location);

    // Add contact geometry.
    m_wheel->SetCollide(true);

    m_wheel->GetCollisionModel()->ClearModel();
    m_wheel->GetCollisionModel()->AddCylinder(getWheelRadius(), getWheelRadius(), getWheelWidth() / 2);
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
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleIdler::AddWheelVisualization(const ChColor& color) {
    double radius = getWheelRadius();
    double width = getWheelWidth();

    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    cyl->GetCylinderGeometry().rad = radius;
    m_wheel->AddAsset(cyl);

    chrono::ChSharedPtr<chrono::ChTexture> tex(new chrono::ChTexture);
    tex->SetTextureFilename(chrono::GetChronoDataFile("bluwhite.png"));
    m_wheel->AddAsset(tex);
}

}  // end namespace vehicle
}  // end namespace chrono
