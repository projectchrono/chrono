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
// Base class for a semi-trailing arm suspension (non-steerable).
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

#include <algorithm>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSemiTrailingArm.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSemiTrailingArm::m_pointNames[] = {"SPINDLE ", "TA_CM",    "TA_O",     "TA_I",    "TA_S",
                                                       "SHOCK_C ", "SHOCK_A ", "SPRING_C", "SPRING_A"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSemiTrailingArm::ChSemiTrailingArm(const std::string& name) : ChSuspension(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSemiTrailingArm::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
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

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, right_ang_vel);
}

void ChSemiTrailingArm::InitializeSide(VehicleSide side,
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

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize trailing arm body.
    // Determine the rotation matrix of the arm based on the plane of the hard points
    // (z axis normal to the plane of the arm)
    w = Vcross(points[TA_O] - points[TA_S], points[TA_I] - points[TA_S]);
    w.Normalize();
    u = points[TA_O] - points[TA_I];
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_arm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm[side]->SetNameString(m_name + "_arm" + suffix);
    m_arm[side]->SetPos(points[TA_CM]);
    m_arm[side]->SetRot(rot);
    m_arm[side]->SetMass(getArmMass());
    m_arm[side]->SetInertiaXX(getArmInertia());
    chassis->GetSystem()->AddBody(m_arm[side]);

    // Create and initialize the revolute joint between arm and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_arm[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and arm.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the arm.
    v = Vcross(points[TA_O] - points[TA_S], points[TA_I] - points[TA_S]);
    v.Normalize();
    w = points[TA_O] - points[TA_I];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_revoluteArm[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteArm[side]->SetNameString(m_name + "_revoluteArm" + suffix);
    m_revoluteArm[side]->Initialize(chassis, m_arm[side],
                                    ChCoordsys<>((points[TA_O] + points[TA_I]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteArm[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_arm[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_arm[side], false, points[SPRING_C], points[SPRING_A], false,
                               getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
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
double ChSemiTrailingArm::GetMass() const {
    return 2 * (getSpindleMass() + getArmMass());
}

// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChSemiTrailingArm::GetCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += getSpindleMass() * m_spindle[LEFT]->GetPos();
    com += getSpindleMass() * m_spindle[RIGHT]->GetPos();

    com += getArmMass() * m_arm[LEFT]->GetPos();
    com += getArmMass() * m_arm[RIGHT]->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSemiTrailingArm::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChSemiTrailingArm::ReportSuspensionForce(VehicleSide side) const {
    ChSuspension::Force force;

    force.spring_force = m_spring[side]->GetForce();
    force.spring_length = m_spring[side]->GetLength();
    force.spring_velocity = m_spring[side]->GetVelocity();

    force.shock_force = m_shock[side]->GetForce();
    force.shock_length = m_shock[side]->GetLength();
    force.shock_velocity = m_shock[side]->GetVelocity();

    return force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSemiTrailingArm::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSemiTrailingArm::LogConstraintViolations(VehicleSide side) {
    {
        ChVectorDynamic<> C = m_revoluteArm[side]->GetC();
        GetLog() << "LCA revolute          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
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
void ChSemiTrailingArm::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for upper control arms
    AddVisualizationArm(m_arm[LEFT], m_pointsL[TA_O], m_pointsL[TA_I], m_pointsL[TA_S], m_pointsL[SPINDLE],
                        getArmRadius());
    AddVisualizationArm(m_arm[RIGHT], m_pointsR[TA_O], m_pointsR[TA_I], m_pointsR[TA_S], m_pointsR[SPINDLE],
                        getArmRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
}

void ChSemiTrailingArm::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_arm[LEFT]->GetAssets().clear();
    m_arm[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSemiTrailingArm::AddVisualizationArm(std::shared_ptr<ChBody> arm,
                                            const ChVector<> pt_AC_O,
                                            const ChVector<> pt_AC_I,
                                            const ChVector<> pt_AS,
                                            const ChVector<> pt_S,
                                            double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_AC_O = arm->TransformPointParentToLocal(pt_AC_O);
    ChVector<> p_AC_I = arm->TransformPointParentToLocal(pt_AC_I);
    ChVector<> p_AS = arm->TransformPointParentToLocal(pt_AS);
    ChVector<> p_S = arm->TransformPointParentToLocal(pt_S);

    auto cyl_O = chrono_types::make_shared<ChCylinderShape>();
    cyl_O->GetCylinderGeometry().p1 = p_AC_O;
    cyl_O->GetCylinderGeometry().p2 = p_AS;
    cyl_O->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_O);

    auto cyl_I = chrono_types::make_shared<ChCylinderShape>();
    cyl_I->GetCylinderGeometry().p1 = p_AC_I;
    cyl_I->GetCylinderGeometry().p2 = p_AS;
    cyl_I->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_I);

    if ((p_AS - p_S).Length2() > threshold2) {
        auto cyl_S = chrono_types::make_shared<ChCylinderShape>();
        cyl_S->GetCylinderGeometry().p1 = p_AS;
        cyl_S->GetCylinderGeometry().p2 = p_S;
        cyl_S->GetCylinderGeometry().rad = radius;
        arm->AddAsset(cyl_S);
    }

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    arm->AddAsset(col);
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSemiTrailingArm::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteArm[0]);
    joints.push_back(m_revoluteArm[1]);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChSemiTrailingArm::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteArm[0]);
    joints.push_back(m_revoluteArm[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
