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
void ChSinglePinShoe::Initialize(ChSystem* system, const ChVector<>& location, const ChQuaternion<>& rotation) {
    // Create the shoe body.
    m_shoe = ChSharedPtr<ChBody>(new ChBody(system->GetContactMethod()));
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetPos(location);
    m_shoe->SetRot(rotation);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    system->AddBody(m_shoe);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSinglePinShoe::Connect(ChSharedPtr<ChTrackShoe> next) {
    // Create and initialize the revolute joint.
    ChVector<> loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetLength() / 2, 0, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngY(CH_C_PI_2);

    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
}

}  // end namespace vehicle
}  // end namespace chrono
