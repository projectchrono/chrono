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
// Template for a rigid tire
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name)
    : ChTire(name), m_friction(0.6f), m_restitution(0.1f), m_young_modulus(2e5f), m_poisson_ratio(0.3f) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetContactMaterial(float friction_coefficient,
                                     float restitution_coefficient,
                                     float young_modulus,
                                     float poisson_ratio) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::Initialize(std::shared_ptr<ChBody> wheel) {
    wheel->SetCollide(true);

    wheel->GetCollisionModel()->ClearModel();
    wheel->GetCollisionModel()->AddCylinder(getRadius(), getRadius(), getWidth() / 2);
    wheel->GetCollisionModel()->BuildModel();

    switch (wheel->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            wheel->GetMaterialSurface()->SetFriction(m_friction);
            wheel->GetMaterialSurface()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurfaceBase::DEM:
            wheel->GetMaterialSurfaceDEM()->SetFriction(m_friction);
            wheel->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
            wheel->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
            wheel->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TireForce ChRigidTire::GetTireForce() const {
    TireForce tire_force;

    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.point = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
