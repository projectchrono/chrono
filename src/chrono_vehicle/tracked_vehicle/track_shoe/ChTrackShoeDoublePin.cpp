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
// Base class for a double-pin track shoe (template definition).
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/ChSubsysDefs.h"
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

    // Set contact material properties.
    switch (m_connector_L->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_connector_L->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_connector_L->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_connector_L->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_connector_L->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_connector_L->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_connector_L->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_connector_L->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_connector_L->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_connector_L->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_connector_L->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }

    switch (m_connector_R->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_connector_R->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            m_connector_R->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            m_connector_R->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            m_connector_R->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            m_connector_R->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            m_connector_R->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            m_connector_R->GetMaterialSurfaceSMC()->SetKn(m_kn);
            m_connector_R->GetMaterialSurfaceSMC()->SetGn(m_gn);
            m_connector_R->GetMaterialSurfaceSMC()->SetKt(m_kt);
            m_connector_R->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }
}

void ChTrackShoeDoublePin::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& loc_shoe,
                                      const ChQuaternion<>& rot_shoe,
                                      const ChVector<>& loc_connector_L,
                                      const ChVector<>& loc_connector_R,
                                      const ChQuaternion<>& rot_connector) {
    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(loc_shoe));
    m_shoe->SetRot(chassis->GetRot() * rot_shoe);

    m_connector_L->SetPos(chassis->TransformPointLocalToParent(loc_connector_L));
    m_connector_L->SetRot(chassis->GetRot() * rot_connector);

    m_connector_R->SetPos(chassis->TransformPointLocalToParent(loc_connector_R));
    m_connector_R->SetRot(chassis->GetRot() * rot_connector);
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
void ChTrackShoeDoublePin::AddShoeContact() {
    double pitch = GetPitch();

    const ChVector<>& pad_dims = GetPadBoxDimensions();
    const ChVector<>& guide_dims = GetGuideBoxDimensions();

    double p0y = 2.1 * (pad_dims.y() / 2);
    double p1y = 1.5 * (pad_dims.y() / 2);

    m_shoe->GetCollisionModel()->ClearModel();

    m_shoe->GetCollisionModel()->SetFamily(TrackedCollisionFamily::SHOES);
    m_shoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);

    m_shoe->GetCollisionModel()->AddBox(pad_dims.x() / 2, pad_dims.y() / 2, pad_dims.z() / 2, GetPadBoxLocation());
    m_shoe->GetCollisionModel()->AddBox(guide_dims.x() / 2, guide_dims.y() / 2, guide_dims.z() / 2, GetGuideBoxLocation());

    m_shoe->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
    AddConnectorVisualization(m_connector_L);
    AddConnectorVisualization(m_connector_R);
}

void ChTrackShoeDoublePin::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
    m_connector_L->GetAssets().clear();
    m_connector_R->GetAssets().clear();
}

void ChTrackShoeDoublePin::AddShoeVisualization() {
    const ChVector<>& pad_box_dims = GetPadBoxDimensions();
    const ChVector<>& guide_box_dims = GetGuideBoxDimensions();
    double s_width = GetShoeWidth();
    double c_width = GetConnectorWidth();

    double pin_radius = pad_box_dims.z() / 6;
    double pin_len = s_width + c_width + 2 * c_width;

    auto rev_rear = std::make_shared<ChCylinderShape>();
    rev_rear->GetCylinderGeometry().p1 = ChVector<>(-0.5 * GetShoeLength(), -0.5 * pin_len, 0);
    rev_rear->GetCylinderGeometry().p2 = ChVector<>(-0.5 * GetShoeLength(), +0.5 * pin_len, 0);
    rev_rear->GetCylinderGeometry().rad = pin_radius;
    m_shoe->AddAsset(rev_rear);

    auto rev_front = std::make_shared<ChCylinderShape>();
    rev_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * GetShoeLength(), -0.5 * pin_len, 0);
    rev_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * GetShoeLength(), +0.5 * pin_len, 0);
    rev_front->GetCylinderGeometry().rad = pin_radius;
    m_shoe->AddAsset(rev_front);

    // Render the pad contact box
    auto box_shoe = std::make_shared<ChBoxShape>();
    box_shoe->GetBoxGeometry().SetLengths(pad_box_dims);
    box_shoe->Pos = GetPadBoxLocation();
    m_shoe->AddAsset(box_shoe);

    // Render the guiding pin contact box
    auto box_pin = std::make_shared<ChBoxShape>();
    box_pin->GetBoxGeometry().SetLengths(guide_box_dims);
    box_pin->Pos = GetGuideBoxLocation();
    m_shoe->AddAsset(box_pin);

    auto col = std::make_shared<ChColorAsset>();
    if (m_index == 0)
        col->SetColor(ChColor(0.6f, 0.3f, 0.3f));
    else if (m_index % 2 == 0)
        col->SetColor(ChColor(0.3f, 0.6f, 0.3f));
    else
        col->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    m_shoe->AddAsset(col);
}

void ChTrackShoeDoublePin::AddConnectorVisualization(std::shared_ptr<ChBody> connector) {
    double c_length = GetConnectorLength();
    double c_width = GetConnectorWidth();
    double c_radius = GetConnectorRadius();

    auto cyl_rear = std::make_shared<ChCylinderShape>();
    cyl_rear->GetCylinderGeometry().p1 = ChVector<>(-0.5 * c_length, -0.5 * c_width, 0);
    cyl_rear->GetCylinderGeometry().p2 = ChVector<>(-0.5 * c_length, +0.5 * c_width, 0);
    cyl_rear->GetCylinderGeometry().rad = c_radius;
    connector->AddAsset(cyl_rear);

    auto cyl_front = std::make_shared<ChCylinderShape>();
    cyl_front->GetCylinderGeometry().p1 = ChVector<>(0.5 * c_length, -0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().p2 = ChVector<>(0.5 * c_length, +0.5 * c_width, 0);
    cyl_front->GetCylinderGeometry().rad = c_radius;
    connector->AddAsset(cyl_front);

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(c_length, c_width, 2 * c_radius));
    box->Pos = ChVector<>(0, 0, 0);
    connector->AddAsset(box);

    auto col = std::make_shared<ChColorAsset>();
    if (m_index == 0)
        col->SetColor(ChColor(0.7f, 0.4f, 0.4f));
    else if (m_index % 2 == 0)
        col->SetColor(ChColor(0.4f, 0.7f, 0.4f));
    else
        col->SetColor(ChColor(0.4f, 0.4f, 0.7f));
    connector->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();

    // Create and initialize the revolute joints between shoe body and connector bodies.
    ChVector<> loc_L = m_shoe->TransformPointLocalToParent(ChVector<>(GetShoeLength() / 2, +GetShoeWidth() / 2, 0));
    ChVector<> loc_R = m_shoe->TransformPointLocalToParent(ChVector<>(GetShoeLength() / 2, -GetShoeWidth() / 2, 0));
    ChQuaternion<> rot = m_shoe->GetRot() * Q_from_AngX(CH_C_PI_2);

    m_revolute_L = std::make_shared<ChLinkLockRevolute>();
    m_revolute_L->SetNameString(m_name + "_rev_this_L");
    m_revolute_L->Initialize(m_shoe, m_connector_L, ChCoordsys<>(loc_L, rot));
    system->AddLink(m_revolute_L);

    m_revolute_R = std::make_shared<ChLinkLockRevolute>();
    m_revolute_R->SetNameString(m_name + "_rev_this_R");
    m_revolute_R->Initialize(m_shoe, m_connector_R, ChCoordsys<>(loc_R, rot));
    system->AddLink(m_revolute_R);

    // Create connections between these connector bodies and the next shoe body
    if (m_index == -1) {
        // Create and initialize a point-line joint for left connector (sliding along X axis)
        loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));
        rot = m_connector_L->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_L = std::make_shared<ChLinkLockPointLine>();
        pointline_L->SetNameString(m_name + "_pointline_next_L");
        pointline_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L, rot));
        system->AddLink(pointline_L);

        // Create and initialize a point-line joint for left connector (sliding along X axis)
        loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));
        rot = m_connector_R->GetRot() * Q_from_AngZ(CH_C_PI_2);

        auto pointline_R = std::make_shared<ChLinkLockPointLine>();
        pointline_R->SetNameString(m_name + "_pointline_next_R");
        pointline_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R, rot));
        system->AddLink(pointline_R);
    } else {
        // Create and initialize a spherical joint for left connector
        loc_L = m_connector_L->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));

        auto sph_L = std::make_shared<ChLinkLockSpherical>();
        sph_L->SetNameString(m_name + "_sph_next_L");
        sph_L->Initialize(next->GetShoeBody(), m_connector_L, ChCoordsys<>(loc_L));
        system->AddLink(sph_L);

        // Create and initialize a spherical joint for left connector
        loc_R = m_connector_R->TransformPointLocalToParent(ChVector<>(GetConnectorLength() / 2, 0, 0));

        auto sph_R = std::make_shared<ChLinkLockSpherical>();
        sph_R->SetNameString(m_name + "_sph_next_R");
        sph_R->Initialize(next->GetShoeBody(), m_connector_R, ChCoordsys<>(loc_R));
        system->AddLink(sph_R);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeDoublePin::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.push_back(m_connector_L);
    bodies.push_back(m_connector_R);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_L);
    joints.push_back(m_revolute_R);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChTrackShoeDoublePin::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    bodies.push_back(m_connector_L);
    bodies.push_back(m_connector_R);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute_L);
    joints.push_back(m_revolute_R);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
