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
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSinglePin.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoeSinglePin::ChTrackShoeSinglePin(const std::string& name) : ChTrackShoe(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
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
            m_shoe->GetMaterialSurfaceDEM()->SetKn(m_kn);
            m_shoe->GetMaterialSurfaceDEM()->SetGn(m_gn);
            m_shoe->GetMaterialSurfaceDEM()->SetKt(m_kt);
            m_shoe->GetMaterialSurfaceDEM()->SetGt(m_gt);
            break;
    }

    AddShoeContact();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChTrackShoeSinglePin::GetMass() const {
    return GetShoeMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::AddShoeContact() {
    double pitch = GetPitch();

    double cyl_radius = GetCylinderRadius();

    const ChVector<>& pad_dims = GetPadBoxDimensions();
    const ChVector<>& guide_dims = GetGuideBoxDimensions();

    double p0y = 2.1 * (pad_dims.y() / 2);
    double p1y = 1.5 * (pad_dims.y() / 2);

    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->AddBox(pad_dims.x() / 2, pad_dims.y() / 2, pad_dims.z() / 2, GetPadBoxLocation());

    m_shoe->GetCollisionModel()->AddBox(guide_dims.x() / 2, guide_dims.y() / 2, guide_dims.z() / 2, GetGuideBoxLocation());

    m_shoe->GetCollisionModel()->AddBox((pitch - 2 * cyl_radius) / 2, 0.95 * (p0y - p1y) / 2, pad_dims.z() / 6,
                                        ChVector<>(0, +0.95 * (p0y + p1y) / 2, 0));
    m_shoe->GetCollisionModel()->AddBox((pitch - 2 * cyl_radius) / 2, 0.95 * (p0y - p1y) / 2, pad_dims.z() / 6,
                                        ChVector<>(0, -0.95 * (p0y + p1y) / 2, 0));

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    double pitch = GetPitch();

    double front_cyl_loc = GetFrontCylinderLoc();
    double rear_cyl_loc = GetRearCylinderLoc();
    double cyl_radius = GetCylinderRadius();

    const ChVector<>& pad_box_dims = GetPadBoxDimensions();
    const ChVector<>& guide_box_dims = GetGuideBoxDimensions();

    double p0y = 2.1 * (pad_box_dims.y() / 2);
    double p1y = 1.5 * (pad_box_dims.y() / 2);
    double p2y = 0.5 * (pad_box_dims.y() / 2);

    // Render the revolute pin
    auto rev_axis = std::make_shared<ChCylinderShape>();
    rev_axis->GetCylinderGeometry().p1 = ChVector<>(pitch / 2, -p0y, 0);
    rev_axis->GetCylinderGeometry().p2 = ChVector<>(pitch / 2, p0y, 0);
    rev_axis->GetCylinderGeometry().rad = cyl_radius / 1.5;
    m_shoe->AddAsset(rev_axis);

    // Render boxes between pins
    auto box_L = std::make_shared<ChBoxShape>();
    box_L->GetBoxGeometry().SetLengths(ChVector<>(pitch - 1.5 * cyl_radius, 0.95 * (p0y - p1y), pad_box_dims.z() / 3));
    box_L->GetBoxGeometry().Pos = ChVector<>(0, +0.95 * (p0y + p1y) / 2, 0);
    m_shoe->AddAsset(box_L);

    auto box_R = std::make_shared<ChBoxShape>();
    box_R->GetBoxGeometry().SetLengths(ChVector<>(pitch - 1.5 * cyl_radius, 0.95 * (p0y - p1y), pad_box_dims.z() / 3));
    box_R->GetBoxGeometry().Pos = ChVector<>(0, -0.95 * (p0y + p1y) / 2, 0);
    m_shoe->AddAsset(box_R);

    // Render the contact cylinders (for contact with sprocket)
    auto cyl_FR = std::make_shared<ChCylinderShape>();
    cyl_FR->GetCylinderGeometry().p1 = ChVector<>(front_cyl_loc, -p1y, 0);
    cyl_FR->GetCylinderGeometry().p2 = ChVector<>(front_cyl_loc, -p2y, 0);
    cyl_FR->GetCylinderGeometry().rad = cyl_radius;
    m_shoe->AddAsset(cyl_FR);

    auto cyl_RR = std::make_shared<ChCylinderShape>();
    cyl_RR->GetCylinderGeometry().p1 = ChVector<>(rear_cyl_loc, -p1y, 0);
    cyl_RR->GetCylinderGeometry().p2 = ChVector<>(rear_cyl_loc, -p2y, 0);
    cyl_RR->GetCylinderGeometry().rad = cyl_radius;
    m_shoe->AddAsset(cyl_RR);

    auto cyl_FL = std::make_shared<ChCylinderShape>();
    cyl_FL->GetCylinderGeometry().p1 = ChVector<>(front_cyl_loc, p1y, 0);
    cyl_FL->GetCylinderGeometry().p2 = ChVector<>(front_cyl_loc, p2y, 0);
    cyl_FL->GetCylinderGeometry().rad = cyl_radius;
    m_shoe->AddAsset(cyl_FL);

    auto cyl_RL = std::make_shared<ChCylinderShape>();
    cyl_RL->GetCylinderGeometry().p1 = ChVector<>(rear_cyl_loc, p1y, 0);
    cyl_RL->GetCylinderGeometry().p2 = ChVector<>(rear_cyl_loc, p2y, 0);
    cyl_RL->GetCylinderGeometry().rad = cyl_radius;
    m_shoe->AddAsset(cyl_RL);

    // Render the pad contact box
    auto box_shoe = std::make_shared<ChBoxShape>();
    box_shoe->GetBoxGeometry().SetLengths(pad_box_dims);
    box_shoe->GetBoxGeometry().Pos = GetPadBoxLocation();
    m_shoe->AddAsset(box_shoe);

    // Render the guiding pin contact box
    auto box_pin = std::make_shared<ChBoxShape>();
    box_pin->GetBoxGeometry().SetLengths(guide_box_dims);
    box_pin->GetBoxGeometry().Pos = GetGuideBoxLocation();
    m_shoe->AddAsset(box_pin);

    // Assign color (based on track shoe index)
    auto col = std::make_shared<ChColorAsset>();
    if (m_index == 0)
        col->SetColor(ChColor(0.6f, 0.3f, 0.3f));
    else if (m_index % 2 == 0)
        col->SetColor(ChColor(0.3f, 0.6f, 0.3f));
    else
        col->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    m_shoe->AddAsset(col);
}

void ChTrackShoeSinglePin::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeSinglePin::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChVector<> loc = m_shoe->TransformPointLocalToParent(ChVector<>(GetPitch() / 2, 0, 0));

    if (m_index == 0) {
        // Create and initialize a point-line joint (sliding line along X)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline = std::make_shared<ChLinkLockPointLine>();
        pointline->SetNameString(m_name + "_pointline");
        pointline->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        m_shoe->GetSystem()->AddLink(pointline);
    } else {
        // Create and initialize the revolute joint (rotation axis along Z)
        ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

        auto revolute = std::make_shared<ChLinkLockRevolute>();
        revolute->SetNameString(m_name + "_revolute");
        revolute->Initialize(m_shoe, next->GetShoeBody(), ChCoordsys<>(loc, rot));
        m_shoe->GetSystem()->AddLink(revolute);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
