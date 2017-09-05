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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band rigid-link track shoe (template definition).
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeRigidCB::ChTrackShoeRigidCB(const std::string& name) : ChTrackShoe(name) {}

void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const ChVector<>& location,
                                    const ChQuaternion<>& rotation) {
    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                    const std::vector<ChCoordsys<>>& component_pos) {
    // Check to make sure that the correct number of coordinate systems were provided for
    // this shoe (tread + associated web segments)
    assert(component_pos.size() == GetNumWebSegments() + 1);

    // Express the tread body's location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(component_pos[0].pos);
    ChQuaternion<> rot = chassis->GetRot() * component_pos[0].rot;

    // Create the tread body
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_tread");
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetTreadMass());
    m_shoe->SetInertiaXX(GetTreadInertia());
    chassis->GetSystem()->AddBody(m_shoe);

    // Add contact geometry.
    m_shoe->SetCollide(true);

    switch (m_shoe->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_shoe->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_shoe->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_shoe->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_shoe->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_shoe->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_shoe->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_shoe->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_shoe->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_shoe->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    AddShoeContact();

    // Cache parameters for a single web segment.
    m_seg_length = GetWebLength() / GetNumWebSegments();
    m_seg_mass = GetWebMass() / GetNumWebSegments();
    m_seg_inertia = GetWebInertia();  //// TODO - properly distribute web inertia

    // Create the required number of web segment bodies
    for (size_t is = 0; is < GetNumWebSegments(); is++) {
        m_web_segments.push_back(std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody()));

        m_web_segments[is]->SetNameString(m_name + "_web_" + std::to_string(is));
        ChVector<> loc = chassis->TransformPointLocalToParent(component_pos[is + 1].pos);
        ChQuaternion<> rot = chassis->GetRot() * component_pos[is + 1].rot;
        m_web_segments[is]->SetPos(loc);
        m_web_segments[is]->SetRot(rot);
        m_web_segments[is]->SetMass(m_seg_mass);
        m_web_segments[is]->SetInertiaXX(m_seg_inertia);
        chassis->GetSystem()->AddBody(m_web_segments[is]);

        // Add contact geometry.
        m_web_segments[is]->SetCollide(true);

        switch (m_web_segments[is]->GetContactMethod()) {
            case ChMaterialSurface::NSC:
                m_web_segments[is]->GetMaterialSurfaceNSC()->SetFriction(m_friction);
                m_web_segments[is]->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
                break;
            case ChMaterialSurface::SMC:
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetFriction(m_friction);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetKn(m_kn);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetGn(m_gn);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetKt(m_kt);
                m_web_segments[is]->GetMaterialSurfaceSMC()->SetGt(m_gt);
                break;
        }

        AddWebContact(m_web_segments[is]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeRigidCB::GetMass() const {
    return GetTreadMass() + GetWebMass();
}

double ChTrackShoeRigidCB::GetPitch() const {
    return GetToothBaseLength() + GetWebLength();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddShoeContact() {
    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    // Guide pin
    ChVector<> g_hdims = GetGuideBoxDimensions() / 2;
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + g_hdims.z());
    m_shoe->GetCollisionModel()->AddBox(g_hdims.x(), g_hdims.y(), g_hdims.z(), g_loc);

    // Main box
    ChVector<> b_hdims(GetTreadLength() / 2, GetBeltWidth() / 2, (GetWebThickness() + GetTreadThickness()) / 2);
    ChVector<> b_loc(0, 0, -GetTreadThickness() / 2);
    m_shoe->GetCollisionModel()->AddBox(b_hdims.x(), b_hdims.y(), b_hdims.z(), b_loc);

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddWebContact(std::shared_ptr<ChBody> segment) {
    segment->GetCollisionModel()->ClearModel();

    segment->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    segment->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    segment->GetCollisionModel()->AddBox(m_seg_length / 2, GetBeltWidth() / 2, GetWebThickness() / 2);

    segment->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    for (auto segment : m_web_segments)
        AddWebVisualization(segment);
}

void ChTrackShoeRigidCB::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
    for (auto segment : m_web_segments) {
        segment->GetAssets().clear();
    }
}

ChColor GetColor(int index) {
    if (index == 0)
        return ChColor(0.7f, 0.4f, 0.4f);
    else if (index % 2 == 0)
        return ChColor(0.4f, 0.7f, 0.4f);
    else
        return ChColor(0.4f, 0.4f, 0.7f);
}

void ChTrackShoeRigidCB::AddShoeVisualization() {
    m_shoe->AddAsset(std::make_shared<ChColorAsset>(GetColor(m_index)));

    // Guide pin
    ChVector<> g_hdims = GetGuideBoxDimensions() / 2;
    ChVector<> g_loc(GetGuideBoxOffsetX(), 0, GetWebThickness() / 2 + g_hdims.z());
    auto box_pin = std::make_shared<ChBoxShape>();
    box_pin->GetBoxGeometry().Size = g_hdims;
    box_pin->GetBoxGeometry().Pos = g_loc;
    m_shoe->AddAsset(box_pin);

    // Main box
    ChVector<> b_hdims(GetTreadLength() / 2, GetBeltWidth() / 2, (GetWebThickness() + GetTreadThickness()) / 2);
    ChVector<> b_loc(0, 0, -GetTreadThickness() / 2);
    auto box_main = std::make_shared<ChBoxShape>();
    box_main->GetBoxGeometry().Size = b_hdims;
    box_main->GetBoxGeometry().Pos = b_loc;
    m_shoe->AddAsset(box_main);

    // Connection to first web segment
    double radius = GetWebThickness() / 4;
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(GetTreadLength() / 2, -GetBeltWidth() / 2 - 2 * radius, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(GetTreadLength() / 2, +GetBeltWidth() / 2 + 2 * radius, 0);
    m_shoe->AddAsset(cyl);
}

void ChTrackShoeRigidCB::AddWebVisualization(std::shared_ptr<ChBody> segment) {
    segment->AddAsset(std::make_shared<ChColorAsset>(GetColor(m_index)));

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(m_seg_length, GetBeltWidth(), GetWebThickness()));
    segment->AddAsset(box);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();
    ChVector<> loc;
    ChQuaternion<> rot;

    // Connect tread body to the first web segment.
    loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, 0, 0));
    rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);
    auto revolute0 = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute0);
    revolute0->SetNameString(m_name + "_revolute_0");
    revolute0->Initialize(m_shoe, m_web_segments[0], ChCoordsys<>(loc, rot));

    // Connect the web segments to each other.
    for (size_t is = 0; is < GetNumWebSegments() - 1; is++) {
        loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
        rot = m_web_segments[is]->GetRot() * Q_from_AngX(CH_C_PI_2);
        auto revolute = std::make_shared<ChLinkLockRevolute>();
        system->AddLink(revolute);
        revolute->SetNameString(m_name + "_revolute_" + std::to_string(is + 1));
        revolute->Initialize(m_web_segments[is], m_web_segments[is + 1], ChCoordsys<>(loc, rot));
    }

    // Connect the last web segment to the tread body from the next track shoe.
    int is = GetNumWebSegments() - 1;
    loc = m_web_segments[is]->TransformPointLocalToParent(ChVector<>(m_seg_length / 2, 0, 0));
    rot = m_web_segments[is]->GetRot() * Q_from_AngX(CH_C_PI_2);
    auto revolute1 = std::make_shared<ChLinkLockRevolute>();
    system->AddLink(revolute1);
    revolute1->SetNameString(m_name + "_revolute");
    revolute1->Initialize(m_web_segments[is], next->GetShoeBody(), ChCoordsys<>(loc, rot));
}

}  // end namespace vehicle
}  // end namespace chrono
