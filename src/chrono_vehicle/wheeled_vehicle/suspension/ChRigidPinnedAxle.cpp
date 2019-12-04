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
// Base class for a rigid suspension with a pinned axle. The rigid axle is pinned
// (with a revolute joint) to the chassis, while the spindle bodies are directly
// attached to the axle body through revolute joints.
//
// This is a dependent, non-steerable suspension template.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidPinnedAxle::ChRigidPinnedAxle(const std::string& name) : ChSuspension(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                   const ChVector<>& location,
                                   std::shared_ptr<ChBody> tierod_body,
                                   int steering_index,
                                   double left_ang_vel,
                                   double right_ang_vel) {
    m_location = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    // Create the rigid axle and its connection to the chassis
    ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(getAxleTubeCOM());
    m_axleTube = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_axleTube->SetNameString(m_name + "_axleTube");
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetSystem()->AddBody(m_axleTube);

    m_axlePinLoc = suspension_to_abs.TransformLocalToParent(getAxlePinLocation());
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();
    ChCoordsys<> rev_csys(m_axlePinLoc, chassisRot * Q_from_AngY(CH_C_PI_2));
    m_axlePin = chrono_types::make_shared<ChLinkLockRevolute>();
    m_axlePin->SetNameString(m_name + "_axlePin");
    m_axlePin->Initialize(m_axleTube, chassis, rev_csys);
    chassis->GetSystem()->AddLink(m_axlePin);

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, right_ang_vel);
}

void ChRigidPinnedAxle::InitializeSide(VehicleSide side,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       const std::vector<ChVector<> >& points,
                                       double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize joints
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngX(CH_C_PI_2));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_axleTube, m_spindle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

// -----------------------------------------------------------------------------
// Get the total mass of the suspension subsystem.
// -----------------------------------------------------------------------------
double ChRigidPinnedAxle::GetMass() const {
    return getAxleTubeMass() + 2 * getSpindleMass();
}

// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChRigidPinnedAxle::GetCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += getSpindleMass() * m_spindle[LEFT]->GetPos();
    com += getSpindleMass() * m_spindle[RIGHT]->GetPos();

    com += getAxleTubeMass() * m_axleTube->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChRigidPinnedAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChRigidPinnedAxle::ReportSuspensionForce(VehicleSide side) const {
    ChSuspension::Force force{0, 0, 0, 0, 0, 0};
    return force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::LogConstraintViolations(VehicleSide side) {
    // Revolute joint
    {
        ChVectorDynamic<> C = m_revolute[side]->GetC();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Express points in local frame of the axle tube
    ChVector<> pSL = m_axleTube->TransformPointParentToLocal(m_pointsL[SPINDLE]);
    ChVector<> pSR = m_axleTube->TransformPointParentToLocal(m_pointsR[SPINDLE]);
    ChVector<> pP = m_axleTube->TransformPointParentToLocal(m_axlePinLoc);

    // Add visualization assets for the axle tube body
    auto cyl1 = chrono_types::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().p1 = pSL;
    cyl1->GetCylinderGeometry().p2 = pSR;
    cyl1->GetCylinderGeometry().rad = getAxleTubeRadius();
    m_axleTube->AddAsset(cyl1);

    static const double threshold2 = 1e-6;
    if (pP.Length2() > threshold2) {
        auto cyl2 = chrono_types::make_shared<ChCylinderShape>();
        cyl2->GetCylinderGeometry().p1 = pP;
        cyl2->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl2->GetCylinderGeometry().rad = getAxleTubeRadius() / 2;
        m_axleTube->AddAsset(cyl2);
    }

    auto cyl3 = chrono_types::make_shared<ChCylinderShape>();
    cyl3->GetCylinderGeometry().p1 = pP - ChVector<>(1.5 * getAxleTubeRadius(), 0, 0);
    cyl3->GetCylinderGeometry().p2 = pP + ChVector<>(1.5 * getAxleTubeRadius(), 0, 0);
    cyl3->GetCylinderGeometry().rad = getAxleTubeRadius();
    m_axleTube->AddAsset(cyl3);

    auto col = chrono_types::make_shared<ChColorAsset>(0.2f, 0.2f, 0.6f);
    m_axleTube->AddAsset(col);
}

void ChRigidPinnedAxle::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_axleTube->GetAssets().clear();
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_axlePin);
    ChPart::ExportJointList(jsonDocument, joints);
}

void ChRigidPinnedAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_axlePin);
    database.WriteJoints(joints);
}

}  // end namespace vehicle
}  // end namespace chrono
