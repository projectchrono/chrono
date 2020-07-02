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
// Base class for a single-A arm suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSingleWishbone.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSingleWishbone::m_pointNames[] = {"SPINDLE ",
                                                      "UPRIGHT ",
                                                      "CA_C    ",
                                                      "CA_U    ",
                                                      "CA_CM   ",
                                                      "STRUT_C ",
                                                      "STRUT_A ",
                                                      "TIEROD_C",
                                                      "TIEROD_U"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSingleWishbone::ChSingleWishbone(const std::string& name) : ChSuspension(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::Initialize(std::shared_ptr<ChChassis> chassis,
                                  std::shared_ptr<ChSubchassis> subchassis,
                                  std::shared_ptr<ChSteering> steering,
                                  const ChVector<>& location,
                                  double left_ang_vel,
                                  double right_ang_vel) {
    m_location = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

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
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis->GetBody(), tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), tierod_body, m_pointsR, right_ang_vel);
}

void ChSingleWishbone::InitializeSide(VehicleSide side,
                                      std::shared_ptr<ChBodyAuxRef> chassis,
                                      std::shared_ptr<ChBody> tierod_body,
                                      const std::vector<ChVector<> >& points,
                                      double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();
    ChVector<> up = chassisRot.GetZaxis();

    // Create and initialize the spindle body
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize the upright body
    m_upright[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upright[side]->SetNameString(m_name + "_upright" + suffix);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertiaMoments());
    m_upright[side]->SetInertiaXY(getUprightInertiaProducts());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Orientation of the control arm. 
    // Y axis along the direction of the control arm (connections to chassis and to upright).
    ChMatrix33<> A;
    ChVector<> v = (points[CA_U] - points[CA_C]).GetNormalized();
    ChVector<> u = Vcross(v, up);
    ChVector<> w = Vcross(u, v);
    A.Set_A_axis(u, v, w);
    ChQuaternion<> rot = A.Get_A_quaternion();

    // Create and initialize the control arm body
    m_control_arm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_control_arm[side]->SetNameString(m_name + "_arm" + suffix);
    m_control_arm[side]->SetPos(points[CA_CM]);
    m_control_arm[side]->SetRot(rot);
    m_control_arm[side]->SetMass(getCAMass());
    m_control_arm[side]->SetInertiaXX(getCAInertiaMoments());
    m_control_arm[side]->SetInertiaXY(getCAInertiaProducts());
    chassis->GetSystem()->AddBody(m_control_arm[side]);

    // Create and initialize the revolute joint between upright and spindle
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side],
                                 ChCoordsys<>(points[SPINDLE], chassisRot * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and CA
    m_revoluteCA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteCA[side]->SetNameString(m_name + "_revoluteCA" + suffix);
    m_revoluteCA[side]->Initialize(chassis, m_control_arm[side],
                                   ChCoordsys<>(points[CA_C], chassisRot * Q_from_AngY(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revoluteCA[side]);

    // Create and initialize the revolute joint between upright and CA
    m_revoluteUA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteUA[side]->SetNameString(m_name + "_revoluteUA" + suffix);
    m_revoluteUA[side]->Initialize(m_control_arm[side], m_upright[side], ChCoordsys<>(points[CA_U], chassisRot));
    chassis->GetSystem()->AddLink(m_revoluteUA[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring-damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_control_arm[side], false, points[STRUT_C], points[STRUT_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

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
double ChSingleWishbone::GetMass() const {
    return 2 * (getSpindleMass() + getCAMass() + getUprightMass());
}

// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChSingleWishbone::GetCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += getSpindleMass() * m_spindle[LEFT]->GetPos();
    com += getSpindleMass() * m_spindle[RIGHT]->GetPos();

    com += getCAMass() * m_control_arm[LEFT]->GetPos();
    com += getCAMass() * m_control_arm[RIGHT]->GetPos();

    com += getUprightMass() * m_upright[LEFT]->GetPos();
    com += getUprightMass() * m_upright[RIGHT]->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSingleWishbone::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChSingleWishbone::ReportSuspensionForce(VehicleSide side) const {
    ChSuspension::Force force;

    force.spring_force = m_shock[side]->GetForce();
    force.spring_length = m_shock[side]->GetLength();
    force.spring_velocity = m_shock[side]->GetVelocity();

    force.shock_force = force.spring_force;
    force.shock_length = force.spring_length;
    force.shock_velocity = force.spring_velocity;

    return force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteCA[side]->GetC();
        GetLog() << "LCA revolute          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revoluteUA[side]->GetC();
        GetLog() << "UCA revolute          ";
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

    // Distance constraint
    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for uprights
    AddVisualizationUpright(m_upright[LEFT], m_pointsL[UPRIGHT] , m_pointsL[SPINDLE], m_pointsL[CA_U],
                            m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], m_pointsR[UPRIGHT], m_pointsR[SPINDLE], m_pointsR[CA_U],
                            m_pointsR[TIEROD_U], getUprightRadius());

    // Add visualization for lower control arms
    AddVisualizationControlArm(m_control_arm[LEFT], m_pointsL[CA_C], m_pointsL[CA_U], getCARadius());
    AddVisualizationControlArm(m_control_arm[RIGHT], m_pointsR[CA_C], m_pointsR[CA_U], getCARadius());

    // Add visualization for the shocks
    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.04, 150, 15));
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.04, 150, 15));
    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());

    // Add visualization for the tie-rods
    m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
    m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
}

void ChSingleWishbone::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_control_arm[LEFT]->GetAssets().clear();
    m_control_arm[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();

    m_distTierod[LEFT]->GetAssets().clear();
    m_distTierod[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
                                                  const ChVector<> pt_C,
                                                  const ChVector<> pt_U,
                                                  double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = arm->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_C;
    cyl->GetCylinderGeometry().p2 = p_U;
    cyl->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl);

    auto cyl_B = chrono_types::make_shared<ChCylinderShape>();
    cyl_B->GetCylinderGeometry().p1 = p_C + ChVector<>(radius, 0, 0);
    cyl_B->GetCylinderGeometry().p2 = p_C - ChVector<>(radius, 0, 0);
    cyl_B->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_B);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.7f, 0.7f, 0.7f));
    arm->AddAsset(col);
}

void ChSingleWishbone::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                               const ChVector<> pt_U,
                                               const ChVector<> pt_S,
                                               const ChVector<> pt_A,
                                               const ChVector<> pt_T,
                                               double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);  // upright center
    ChVector<> p_S = upright->TransformPointParentToLocal(pt_S);  // spindle center
    ChVector<> p_A = upright->TransformPointParentToLocal(pt_A);  // connection to arm
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);  // connection to tierod

    if ((p_U - p_S).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_U;
        cyl->GetCylinderGeometry().p2 = p_S;
        cyl->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl);
    }

    if ((p_U - p_A).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_U;
        cyl->GetCylinderGeometry().p2 = p_A;
        cyl->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl);
    }

    if ((p_U - p_T).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_U;
        cyl->GetCylinderGeometry().p2 = p_T;
        cyl->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl);
    }

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_control_arm[0]);
    bodies.push_back(m_control_arm[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteCA[0]);
    joints.push_back(m_revoluteCA[1]);
    joints.push_back(m_revoluteUA[0]);
    joints.push_back(m_revoluteUA[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}


void ChSingleWishbone::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_control_arm[0]);
    bodies.push_back(m_control_arm[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteCA[0]);
    joints.push_back(m_revoluteCA[1]);
    joints.push_back(m_revoluteUA[0]);
    joints.push_back(m_revoluteUA[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
