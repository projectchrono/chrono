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
// Base class for a tracked vehicle sprocket. A sprocket is responsible for
// contact processing with the track shoes of the containing track assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono/assets/ChLineShape.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"

namespace chrono {
namespace vehicle {

void ChSprocket::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // The sprocket reference frame is aligned with that of the chassis and centered at the
    // specified location.
    ////ChFrame<> sprocket_to_abs(location);
    ////sprocket_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
    ChVector<> loc = chassis->GetFrame_REF_to_abs().TransformPointLocalToParent(location);
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();
    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    // Create and initialize the gear body (same orientation as the chassis).
    m_gear = ChSharedPtr<ChBody>(new ChBody(chassis->GetSystem()->GetContactMethod()));
    m_gear->SetNameString(m_name + "_gear");
    m_gear->SetPos(loc);
    m_gear->SetRot(chassisRot);
    m_gear->SetMass(getGearMass());
    m_gear->SetInertiaXX(getGearInertia());
    chassis->GetSystem()->AddBody(m_gear);

    // Add collision shape to gear body.
    double sep = getSeparation();
    ChSharedPtr<geometry::ChLinePath> profile = getProfile();
    m_gear->SetCollide(true);
    m_gear->GetCollisionModel()->SetSafeMargin(0.02);
    m_gear->GetCollisionModel()->ClearModel();
    m_gear->GetCollisionModel()->Add2Dpath(*profile.get_ptr(), ChVector<>(0, 0, sep / 2), rot_y2z);
    m_gear->GetCollisionModel()->Add2Dpath(*profile.get_ptr(), ChVector<>(0, 0, -sep / 2), rot_y2z);
    m_gear->GetCollisionModel()->BuildModel();

    // Create and initialize the revolute joint between chassis and gear.
    ChCoordsys<> rev_csys(loc, chassisRot * y2z);
    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_gear, rev_csys);

    // Create and initialize the axle shaft and its connection to the gear. Note that the
    // gear rotates about the Y axis.
    m_axle = ChSharedPtr<ChShaft>(new ChShaft);
    m_axle->SetNameString(m_name + "_axle");
    m_axle->SetInertia(getAxleInertia());
    chassis->GetSystem()->Add(m_axle);

    m_axle_to_spindle = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
    m_axle_to_spindle->SetNameString(m_name + "_axle_to_spindle");
    m_axle_to_spindle->Initialize(m_axle, m_gear, ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle);
}

void ChSprocket::AddGearVisualization(const ChColor& color) {
    ChQuaternion<> y2z = Q_from_AngX(CH_C_PI_2);
    ChMatrix33<> rot_y2z(y2z);

    double sep = getSeparation();
    ChSharedPtr<geometry::ChLinePath> profile = getProfile();

    ChSharedPtr<ChLineShape> asset_1(new ChLineShape);
    asset_1->SetLineGeometry(profile);
    asset_1->Pos = ChVector<>(0, 0, sep / 2);
    asset_1->Rot = rot_y2z;
    asset_1->SetColor(color);
    m_gear->AddAsset(asset_1);

    ChSharedPtr<ChLineShape> asset_2(new ChLineShape);
    asset_2->SetLineGeometry(profile);
    asset_2->Pos = ChVector<>(0, 0, -sep / 2);
    asset_2->Rot = rot_y2z;
    asset_2->SetColor(color);
    m_gear->AddAsset(asset_2);
}

void ChSprocket::ApplyAxleTorque(double torque) {
    //// TODO: is this really needed?
    //// (the axle is connected to the driveline, so torque is automatically transmitted)
    m_axle->SetAppliedTorque(torque);
}

}  // end namespace vehicle
}  // end namespace chrono
