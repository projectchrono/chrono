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

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRigidPinnedAxle::ChRigidPinnedAxle(const std::string& name) : ChSuspension(name) {}

ChRigidPinnedAxle::~ChRigidPinnedAxle() {
    if (!m_initialized)
        return;

    auto sys = m_axleTube->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_axleTube);
    sys->Remove(m_axlePin);
}

// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::Construct(std::shared_ptr<ChChassis> chassis,
                                  std::shared_ptr<ChSubchassis> subchassis,
                                  std::shared_ptr<ChSteering> steering,
                                  const ChVector3d& location,
                                  double left_ang_vel,
                                  double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create the rigid axle and its connection to the chassis
    ChVector3d axleCOM = suspension_to_abs.TransformPointLocalToParent(getAxleTubeCOM());
    m_axleTube = chrono_types::make_shared<ChBody>();
    m_axleTube->SetName(m_name + "_axleTube");
    m_axleTube->SetTag(m_obj_tag);
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    m_axlePinLoc = suspension_to_abs.TransformPointLocalToParent(getAxlePinLocation());
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();
    m_axlePin = chrono_types::make_shared<ChLinkLockRevolute>();
    m_axlePin->SetName(m_name + "_axlePin");
    m_axlePin->SetTag(m_obj_tag);
    m_axlePin->Initialize(m_axleTube, chassis->GetBody(),
                          ChFrame<>(m_axlePinLoc, chassisRot * QuatFromAngleY(CH_PI_2)));
    chassis->GetBody()->GetSystem()->AddLink(m_axlePin);

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis->GetBody(), m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), m_pointsR, right_ang_vel);
}

void ChRigidPinnedAxle::InitializeSide(VehicleSide side,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       const std::vector<ChVector3d>& points,
                                       double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrameRefToAbs().GetRot();

    // Initialize spindle body (same orientation as the chassis)
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize joints
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_axleTube, m_spindle[side],
                                 ChFrame<>(points[SPINDLE], chassisRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetName(m_name + "_axle" + suffix);
    m_axle[side]->SetTag(m_obj_tag);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPosDt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftBodyRotation>();
    m_axle_to_spindle[side]->SetName(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChRigidPinnedAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + 2 * getSpindleMass();
}

void ChRigidPinnedAxle::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTube->GetFrameCOMToAbs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
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
std::vector<ChSuspension::ForceTSDA> ChRigidPinnedAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces;
    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::LogConstraintViolations(VehicleSide side) {
    // Revolute joint
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        std::cout << "Spindle revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Express points in local frame of the axle tube
    ChVector3d pSL = m_axleTube->TransformPointParentToLocal(m_pointsL[SPINDLE]);
    ChVector3d pSR = m_axleTube->TransformPointParentToLocal(m_pointsR[SPINDLE]);
    ChVector3d pP = m_axleTube->TransformPointParentToLocal(m_axlePinLoc);

    // Add visualization assets for the axle tube body
    utils::ChBodyGeometry::AddVisualizationCylinder(m_axleTube, pSL, pSR, getAxleTubeRadius());

    static const double threshold2 = 1e-6;
    if (pP.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(m_axleTube, pP, VNULL, getAxleTubeRadius() / 2);
    }

    utils::ChBodyGeometry::AddVisualizationCylinder(m_axleTube,                                        //
                                                    pP - ChVector3d(1.5 * getAxleTubeRadius(), 0, 0),  //
                                                    ChVector3d(1.5 * getAxleTubeRadius(), 0, 0),       //
                                                    getAxleTubeRadius());
}

void ChRigidPinnedAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidPinnedAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_axlePin);
    ExportJointList(jsonDocument, joints);
}

void ChRigidPinnedAxle::Output(ChOutput& database) const {
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
