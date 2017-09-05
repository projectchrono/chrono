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
                                    const std::vector<ChCoordsys<>> shoe_components_coordsys) {
    // Check to make sure that the correct number of coordinate systems were provided for
    // this shoe (tread + associated web segments)
    assert(shoe_components_coordsys.size() == GetNumWebSegments() + 1);

    // Express the tread body's location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(shoe_components_coordsys[0].pos);
    ChQuaternion<> rot = chassis->GetRot() * shoe_components_coordsys[0].rot;

    // Create the tread body
    m_shoe = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shoe->SetNameString(m_name + "_tread");
    m_shoe->SetPos(loc);
    m_shoe->SetRot(rot);
    m_shoe->SetMass(GetShoeMasses()[0]);
    m_shoe->SetInertiaXX(GetShoeInertias()[0]);
    chassis->GetSystem()->AddBody(m_shoe);

    //// TODO
    // Add contact geometry.
    // m_shoe->SetCollide(true);

    // switch (m_shoe->GetContactMethod()) {
    // case ChMaterialSurface::NSC:
    //    m_shoe->GetMaterialSurfaceNSC()->SetFriction(m_friction);
    //    m_shoe->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
    //    break;
    // case ChMaterialSurface::SMC:
    //    m_shoe->GetMaterialSurfaceSMC()->SetFriction(m_friction);
    //    m_shoe->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
    //    m_shoe->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
    //    m_shoe->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
    //    m_shoe->GetMaterialSurfaceSMC()->SetKn(m_kn);
    //    m_shoe->GetMaterialSurfaceSMC()->SetGn(m_gn);
    //    m_shoe->GetMaterialSurfaceSMC()->SetKt(m_kt);
    //    m_shoe->GetMaterialSurfaceSMC()->SetGt(m_gt);
    //    break;
    //}

    //// TODO - AddShoeContact();

    //// TODO - Adjust to handle more than 1 web segment;
    // Create the web bodies
    for (size_t web = 0; web < GetNumWebSegments(); web++) {
        m_web_segments.push_back(std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody()));

        char web_idx_str[50];
        sprintf(web_idx_str, "%d", web);
        m_web_segments[web]->SetNameString(m_name + "_web" + web_idx_str);
        ChVector<> loc = chassis->TransformPointLocalToParent(shoe_components_coordsys[web + 1].pos);
        ChQuaternion<> rot = chassis->GetRot() * shoe_components_coordsys[web + 1].rot;

        m_web_segments[web]->SetMass(GetShoeMasses()[web + 1]);
        m_web_segments[web]->SetInertiaXX(GetShoeInertias()[web + 1]);
        chassis->GetSystem()->AddBody(m_web_segments[web]);

        // Set contact material properties.
        switch (m_web_segments[web]->GetContactMethod()) {
            case ChMaterialSurface::NSC:
                m_web_segments[web]->GetMaterialSurfaceNSC()->SetFriction(m_friction);
                m_web_segments[web]->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
                break;
            case ChMaterialSurface::SMC:
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetFriction(m_friction);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetKn(m_kn);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetGn(m_gn);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetKt(m_kt);
                m_web_segments[web]->GetMaterialSurfaceSMC()->SetGt(m_gt);
                break;
        }
    }

    AddWebContact();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeRigidCB::GetMass() const {
    auto shoe_masses = GetShoeMasses();
    double mass = 0;
    for (size_t i = 0; i < shoe_masses.size(); i++) {
        mass += shoe_masses[i];
    }
    return mass;
}

double ChTrackShoeRigidCB::GetPitch() const {
    auto web_lengths = GetWebLengths();
    double pitch = GetToothBaseLength();
    for (size_t i = 0; i < web_lengths.size(); i++) {
        pitch += web_lengths[i];
    }
    return pitch;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddShoeContact() {
    double pitch = GetPitch();

    const ChVector<>& guide_dims = GetGuideBoxDimensions();
    ChVector<> guide_loc(GetGuideBoxOffsetX(), 0, guide_dims.z() / 2);

    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    m_shoe->GetCollisionModel()->AddBox(guide_dims.x() / 2, guide_dims.y() / 2, guide_dims.z() / 2, guide_loc);

    //// TODO - Add other shoe geometry

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddWebContact() {
    auto web_lengths = GetWebLengths();

    for (size_t web = 0; web < GetNumWebSegments(); web++) {
        ChVector<> web_location(0, 0, -GetWebThickness() / 2);

        m_web_segments[web]->GetCollisionModel()->ClearModel();

        m_web_segments[web]->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
        m_web_segments[web]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

        double l = web_lengths[web] / 2;
        double w = GetBeltWidth() / 2;
        double h = GetWebThickness() / 2;
        m_web_segments[web]->GetCollisionModel()->AddBox(web_lengths[web] / 2, GetBeltWidth() / 2,
                                                         GetWebThickness() / 2 * 10, web_location);

        m_web_segments[web]->GetCollisionModel()->BuildModel();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    AddWebVisualization();
}

void ChTrackShoeRigidCB::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
    for (size_t web = 0; web < GetNumWebSegments(); web++) {
        m_web_segments[web]->GetAssets().clear();
    }
}

void ChTrackShoeRigidCB::AddShoeVisualization() {
    //// TODO - only guide pin has been added so far
    const ChVector<>& guide_dims = GetGuideBoxDimensions();
    ChVector<> guide_loc(GetGuideBoxOffsetX(), 0, guide_dims.z() / 2);

    auto col = std::make_shared<ChColorAsset>();
    if (m_index == 0)
        col->SetColor(ChColor(0.7f, 0.4f, 0.4f));
    else if (m_index % 2 == 0)
        col->SetColor(ChColor(0.4f, 0.7f, 0.4f));
    else
        col->SetColor(ChColor(0.4f, 0.4f, 0.7f));
    m_shoe->AddAsset(col);

    // Render the guiding pin contact box
    auto box_pin = std::make_shared<ChBoxShape>();
    box_pin->GetBoxGeometry().SetLengths(guide_dims);
    box_pin->GetBoxGeometry().Pos = guide_loc;
    m_shoe->AddAsset(box_pin);
}

void ChTrackShoeRigidCB::AddWebVisualization() {
    auto web_lengths = GetWebLengths();

    for (size_t web = 0; web < GetNumWebSegments(); web++) {
        auto col = std::make_shared<ChColorAsset>();
        if (m_index == 0)
            col->SetColor(ChColor(0.7f, 0.4f, 0.4f));
        else if (m_index % 2 == 0)
            col->SetColor(ChColor(0.4f, 0.7f, 0.4f));
        else
            col->SetColor(ChColor(0.4f, 0.4f, 0.7f));

        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(ChVector<>(web_lengths[web], GetBeltWidth(), GetWebThickness()));
        box->GetBoxGeometry().Pos = ChVector<>(0, 0, -GetWebThickness() / 2);
        m_web_segments[web]->AddAsset(box);

        m_web_segments[web]->AddAsset(col);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeRigidCB::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();

    // Add the joint for the tread body
    // A point-line joint is created for the first joint so that redundant constraints are not created
    if (m_index == 0) {
        ChVector<> loc =
            m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, -GetBushingDepth(), 0));
        // Create and initialize a point-line joint (sliding line along X)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline = std::make_shared<ChLinkLockPointLine>();
        pointline->SetNameString(m_name + "_pointline");
        if (GetNumWebSegments() < 1) {
            pointline->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        } else {
            pointline->Initialize(m_shoe, m_web_segments[0], ChCoordsys<>(loc, rot));
        }

        system->AddLink(pointline);
    } else {
        ChVector<> loc =
            m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, -GetBushingDepth(), 0));
        // Create and initialize the revolute joint (rotation axis along Z)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

        auto revolute = std::make_shared<ChLinkLockRevolute>();
        revolute->SetNameString(m_name + "_revolute");

        if (GetNumWebSegments() < 1) {
            revolute->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        } else {
            revolute->Initialize(m_shoe, m_web_segments[0], ChCoordsys<>(loc, rot));
        }
        system->AddLink(revolute);
    }

    // Add joints for all of the webs
    auto web_lengths = GetWebLengths();

    for (size_t web = 0; web < GetNumWebSegments(); web++) {
        ChVector<> loc = m_shoe->TransformPointLocalToParent(ChVector<>(web_lengths[web] / 2, -GetBushingDepth(), 0));
        // Create and initialize the revolute joint (rotation axis along Z)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

        auto revolute = std::make_shared<ChLinkLockRevolute>();
        revolute->SetNameString(m_web_segments[web]->GetNameString() + "_revolute");

        if (web == (GetNumWebSegments() - 1)) {
            revolute->Initialize(m_web_segments[web], next->GetShoeBody(), ChCoordsys<>(loc, rot));
        } else {
            revolute->Initialize(m_web_segments[web], m_web_segments[web + 1], ChCoordsys<>(loc, rot));
        }
        system->AddLink(revolute);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
