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
// Base class for a double-pin track shoe (template definition).
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeDoublePin::ChTrackShoeDoublePin(const std::string& name) : ChTrackShoe(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& location,
                                      const ChQuaternion<>& rotation) {
    // Express the track shoe location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    ChVector<> xdir = rot.GetXaxis();
    ChVector<> ydir = rot.GetYaxis();

    // Create the shoe body
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetPos(loc - (0.5 * GetConnectorLength()) * xdir);
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

    // Add visualization to the shoe body.
    AddShoeVisualization();
    
    // Create the connector bodies.
    m_connector_L = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_connector_L->SetNameString(m_name + "_connector_L");
    m_connector_L->SetPos(loc + (0.5 * GetShoeLength()) * xdir + (0.5 * GetShoeWidth()) * ydir);
    m_connector_L->SetRot(rot);
    m_connector_L->SetMass(GetConnectorMass());
    m_connector_L->SetInertiaXX(GetConnectorInertia());
    chassis->GetSystem()->AddBody(m_connector_L);

    m_connector_R = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_connector_R->SetNameString(m_name + "_connector_R");
    m_connector_R->SetPos(loc + (0.5 * GetShoeLength()) * xdir - (0.5 * GetShoeWidth()) * ydir);
    m_connector_R->SetRot(rot);
    m_connector_R->SetMass(GetConnectorMass());
    m_connector_R->SetInertiaXX(GetConnectorInertia());
    chassis->GetSystem()->AddBody(m_connector_R);

    // Add visualization to the connector bodies.
    AddConnectorVisualization(m_connector_L);
    AddConnectorVisualization(m_connector_R);
}

void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& loc_shoe,
                                      const ChQuaternion<>& rot_shoe,
                                      const ChVector<>& loc_connector_L,
                                      const ChVector<>& loc_connector_R,
                                      const ChQuaternion<>& rot_connector) {
    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite body locations and orientations.
    m_shoe->SetPos(loc_shoe);
    m_shoe->SetRot(rot_shoe);
    m_connector_L->SetPos(loc_connector_L);
    m_connector_L->SetRot(rot_connector);
    m_connector_R->SetPos(loc_connector_R);
    m_connector_R->SetRot(rot_connector);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeDoublePin::GetMass() const {
    return GetShoeMass() + 2 * GetConnectorMass();
}

double ChTrackShoeDoublePin::GetPitch() const {
    return GetShoeLength() + GetConnectorLength();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next) {
    // Create and initialize the revolute joints between shoe body and connector bodies.
    ChVector<> loc_L = m_shoe->TransformPointLocalToParent(ChVector<>(GetShoeLength() / 2, +GetShoeWidth() / 2, 0));
    ChVector<> loc_R = m_shoe->TransformPointLocalToParent(ChVector<>(GetShoeLength() / 2, -GetShoeWidth() / 2, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_rev_this_L = std::make_shared<ChLinkLockRevolute>();
    m_rev_this_L->SetNameString(m_name + "_rev_this_L");
    m_rev_this_L->Initialize(m_shoe, m_connector_L, ChCoordsys<>(loc_L, rot));
    m_shoe->GetSystem()->AddLink(m_rev_this_L);

    m_rev_this_R = std::make_shared<ChLinkLockRevolute>();
    m_rev_this_R->SetNameString(m_name + "_rev_this_R");
    m_rev_this_R->Initialize(m_shoe, m_connector_R, ChCoordsys<>(loc_R, rot));
    m_shoe->GetSystem()->AddLink(m_rev_this_R);

    // Create and initialize the revolute joints between connector bodies and next shoe body.
    loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));
    rot = m_connector_L->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_rev_next_L = std::make_shared<ChLinkLockRevolute>();
    m_rev_next_L->SetNameString(m_name + "_rev_next_L");
    m_rev_next_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L, rot));
    m_shoe->GetSystem()->AddLink(m_rev_next_L);

    loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));
    rot = m_connector_R->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_rev_next_R = std::make_shared<ChLinkLockRevolute>();
    m_rev_next_R->SetNameString(m_name + "_rev_next_R");
    m_rev_next_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R, rot));
    m_shoe->GetSystem()->AddLink(m_rev_next_R);
}

}  // end namespace vehicle
}  // end namespace chrono
