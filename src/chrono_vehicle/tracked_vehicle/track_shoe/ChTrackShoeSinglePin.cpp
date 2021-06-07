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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLinkRotSpringCB.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySinglePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeSinglePin::ChTrackShoeSinglePin(const std::string& name) : ChTrackShoeSegmented(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& location,
                                      const ChQuaternion<>& rotation) {
    ChSystem* sys = chassis->GetSystem();

    // Create the shoe body.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    m_shoe = std::shared_ptr<ChBody>(sys->NewBody());
    m_shoe->SetNameString(m_name + "_shoe");
    m_shoe->SetIdentifier(BodyID::SHOE_BODY);
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMass());
    m_shoe->SetInertiaXX(GetShoeInertia());
    m_shoe->SetCollide(true);
    chassis->GetSystem()->AddBody(m_shoe);

    // Create all contact materials
    CreateContactMaterials(sys->GetContactMethod());

    // Add contact geometry on shoe body
    m_geometry.AddCollisionShapes(m_shoe, TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeSinglePin::GetMass() const {
    return GetShoeMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Connect(std::shared_ptr<ChTrackShoe> next, ChTrackAssembly* assembly, bool ccw) {
    auto track = static_cast<ChTrackAssemblySinglePin*>(assembly);
    ChSystem* system = m_shoe->GetSystem();
    double sign = ccw ? +1 : -1;

    bool add_RSDA = (track->GetConnectionType() == ChTrackAssemblySegmented::ConnectionType::RSDA_JOINT);
    assert(!add_RSDA || track->GetTorqueFunctor());

    ChVector<> loc = m_shoe->TransformPointLocalToParent(ChVector<>(sign * GetPitch() / 2, 0, 0));

    if (m_index == 0) {
        // Create and initialize a point-line joint (sliding line along X)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline = chrono_types::make_shared<ChLinkLockPointLine>();
        pointline->SetNameString(m_name + "_pointline");
        pointline->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        system->AddLink(pointline);
    } else {
        // Create and initialize the revolute joint (rotation axis along Z)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->SetNameString(m_name + "_revolute");
        revolute->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        system->AddLink(revolute);
    }

    // Optionally, include rotational spring-damper to model track bending stiffness
    if (add_RSDA) {
        auto rsda = chrono_types::make_shared<ChLinkRotSpringCB>();
        rsda->SetNameString(m_name + "_rsda");
        rsda->Initialize(m_shoe, next->GetShoeBody(), false, ChCoordsys<>(loc, m_shoe->GetRot()),
                         ChCoordsys<>(loc, next->GetShoeBody()->GetRot()));
        rsda->RegisterTorqueFunctor(track->GetTorqueFunctor());
        system->AddLink(rsda);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    ChPart::ExportBodyList(jsonDocument, bodies);
}

void ChTrackShoeSinglePin::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    database.WriteBodies(bodies);
}

}  // end namespace vehicle
}  // end namespace chrono
