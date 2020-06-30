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
// Base class for a tracked vehicle sprocket. A sprocket is responsible for
// contact processing with the track shoes of the containing track assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSprocket::ChSprocket(const std::string& name) : ChPart(name), m_lateral_contact(true) {}

ChSprocket::~ChSprocket() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocket::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location, ChTrackAssembly* track) {
    // The sprocket reference frame is aligned with that of the chassis and centered at the
    // specified location.
    ////ChFrame<> sprocket_to_abs(location);
    ////sprocket_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
    ChVector<> loc = chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(location);
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();
    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    // Create and initialize the gear body (same orientation as the chassis).
    m_gear = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_gear->SetNameString(m_name + "_gear");
    m_gear->SetIdentifier(BodyID::SPROCKET_BODY);
    m_gear->SetPos(loc);
    m_gear->SetRot(chassisRot);
    m_gear->SetMass(GetGearMass());
    m_gear->SetInertiaXX(GetGearInertia());
    chassis->GetSystem()->AddBody(m_gear);

    // Create and initialize the revolute joint between chassis and gear.
    ChCoordsys<> rev_csys(loc, chassisRot * y2z);
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_gear, rev_csys);
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the axle shaft and its connection to the gear. Note that the
    // gear rotates about the Y axis.
    m_axle = chrono_types::make_shared<ChShaft>();
    m_axle->SetNameString(m_name + "_axle");
    m_axle->SetInertia(GetAxleInertia());
    chassis->GetSystem()->Add(m_axle);

    m_axle_to_spindle = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle->SetNameString(m_name + "_axle_to_spindle");
    m_axle_to_spindle->Initialize(m_axle, m_gear, ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle);

    // Enable contact for the gear body and set contact material properties.
    m_gear->SetCollide(true);
    CreateContactMaterial(chassis->GetSystem()->GetContactMethod());

    // Set user-defined custom collision callback class for sprocket-shoes contact.
    chassis->GetSystem()->RegisterCustomCollisionCallback(GetCollisionCallback(track));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSprocket::GetMass() const {
    return GetGearMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocket::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    double sep = GetSeparation();
    std::shared_ptr<geometry::ChLinePath> profile = GetProfile();

    auto asset_1 = chrono_types::make_shared<ChLineShape>();
    asset_1->SetLineGeometry(profile);
    asset_1->Pos = ChVector<>(0, sep / 2, 0);
    asset_1->Rot = rot_y2z;
    asset_1->SetColor(ChColor(1, 0, 0));
    m_gear->AddAsset(asset_1);

    auto asset_2 = chrono_types::make_shared<ChLineShape>();
    asset_2->SetLineGeometry(profile);
    asset_2->Pos = ChVector<>(0, -sep / 2, 0);
    asset_2->Rot = rot_y2z;
    asset_2->SetColor(ChColor(1, 0, 0));
    m_gear->AddAsset(asset_2);
}

void ChSprocket::RemoveVisualizationAssets() {
    m_gear->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocket::ApplyAxleTorque(double torque) {
    //// TODO: is this really needed?
    //// (the axle is connected to the driveline, so torque is automatically transmitted)
    m_axle->SetAppliedTorque(torque);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocket::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetC();
    GetLog() << "  Sprocket-chassis revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocket::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_gear);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChSprocket::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_gear);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
