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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChSinglePinShoe.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSinglePinShoe::ChSinglePinShoe(const std::string& name) : ChTrackShoe(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSinglePinShoe::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                 const ChVector<>& location,
                                 const ChQuaternion<>& rotation) {
    // Create the shoe body.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry.
    m_shoe->SetCollide(true);

    switch (m_shoe->GetContactMethod()) {
    case ChMaterialSurfaceBase::DVI:
        m_shoe->GetMaterialSurface()->SetFriction(m_friction);
        m_shoe->GetMaterialSurface()->SetRestitution(m_restitution);
        break;
    case ChMaterialSurfaceBase::DEM:
        m_shoe->GetMaterialSurfaceDEM()->SetFriction(m_friction);
        m_shoe->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
        m_shoe->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
        m_shoe->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
        break;
    }

    AddShoeContact();

    // Add visualization of the track shoe.
    AddShoeVisualization();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSinglePinShoe::GetMass() const {
    return GetShoeMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSinglePinShoe::Connect(std::shared_ptr<ChTrackShoe> next) {
    // Create and initialize the revolute joint.
    ChVector<> loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetPitch() / 2, 0, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
    m_shoe->GetSystem()->AddLink(m_revolute);
}

}  // end namespace vehicle
}  // end namespace chrono
