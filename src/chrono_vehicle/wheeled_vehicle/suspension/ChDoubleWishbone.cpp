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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Base class for a double-A arm suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChDoubleWishbone::m_pointNames[] = {"SPINDLE ",
                                                      "UPRIGHT ",
                                                      "UCA_F   ",
                                                      "UCA_B   ",
                                                      "UCA_U   ",
                                                      "UCA_CM  ",
                                                      "LCA_F   ",
                                                      "LCA_B   ",
                                                      "LCA_U   ",
                                                      "LCA_CM  ",
                                                      "SHOCK_C ",
                                                      "SHOCK_A ",
                                                      "SPRING_C",
                                                      "SPRING_A",
                                                      "TIEROD_C",
                                                      "TIEROD_U"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishbone::ChDoubleWishbone(const std::string& name, bool vehicle_frame_inertia)
    : ChSuspension(name), m_vehicle_frame_inertia(vehicle_frame_inertia) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::Initialize(std::shared_ptr<ChChassis> chassis,
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

void ChDoubleWishbone::InitializeSide(VehicleSide side,
                                      std::shared_ptr<ChBodyAuxRef> chassis,
                                      std::shared_ptr<ChBody> tierod_body,
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

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upright[side]->SetNameString(m_name + "_upright" + suffix);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia =
            TransformInertiaMatrix(getUprightInertiaMoments(), getUprightInertiaProducts(), chassisRot, chassisRot);
        m_upright[side]->SetInertia(inertia);
    } else {
        m_upright[side]->SetInertiaXX(getUprightInertiaMoments());
        m_upright[side]->SetInertiaXY(getUprightInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize Upper Control Arm body.
    // Determine the rotation matrix of the UCA based on the plane of the hard points
    // (z axis normal to the plane of the UCA)
    w = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
    w.Normalize();
    u = points[UCA_F] - points[UCA_B];
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_UCA[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_UCA[side]->SetNameString(m_name + "_UCA" + suffix);
    m_UCA[side]->SetPos(points[UCA_CM]);
    m_UCA[side]->SetRot(rot);
    m_UCA[side]->SetMass(getUCAMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getUCAInertiaMoments(), getUCAInertiaProducts(), chassisRot, rot);
        m_UCA[side]->SetInertia(inertia);
    } else {
        m_UCA[side]->SetInertiaXX(getUCAInertiaMoments());
        m_UCA[side]->SetInertiaXY(getUCAInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_UCA[side]);

    // Create and initialize Lower Control Arm body.
    // Determine the rotation matrix of the LCA, based on the plane of the hard points
    // (z axis normal to the plane of the LCA)
    w = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
    w.Normalize();
    u = points[LCA_F] - points[LCA_B];
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_LCA[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_LCA[side]->SetNameString(m_name + "_LCA" + suffix);
    m_LCA[side]->SetPos(points[LCA_CM]);
    m_LCA[side]->SetRot(rot);
    m_LCA[side]->SetMass(getLCAMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getLCAInertiaMoments(), getLCAInertiaProducts(), chassisRot, rot);
        m_LCA[side]->SetInertia(inertia);
    } else {
        m_LCA[side]->SetInertiaXX(getLCAInertiaMoments());
        m_LCA[side]->SetInertiaXY(getLCAInertiaProducts());
    }
    chassis->GetSystem()->AddBody(m_LCA[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and UCA.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the UCA.
    v = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
    v.Normalize();
    w = points[UCA_F] - points[UCA_B];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_revoluteUCA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteUCA[side]->SetNameString(m_name + "_revoluteUCA" + suffix);
    m_revoluteUCA[side]->Initialize(chassis, m_UCA[side],
                                    ChCoordsys<>((points[UCA_F] + points[UCA_B]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteUCA[side]);

    // Create and initialize the spherical joint between upright and UCA.
    m_sphericalUCA[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalUCA[side]->SetNameString(m_name + "_sphericalUCA" + suffix);
    m_sphericalUCA[side]->Initialize(m_UCA[side], m_upright[side], ChCoordsys<>(points[UCA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUCA[side]);

    // Create and initialize the revolute joint between chassis and LCA.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the LCA.
    v = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
    v.Normalize();
    w = points[LCA_F] - points[LCA_B];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_revoluteLCA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteLCA[side]->SetNameString(m_name + "_revoluteLCA" + suffix);
    m_revoluteLCA[side]->Initialize(chassis, m_LCA[side],
                                    ChCoordsys<>((points[LCA_F] + points[LCA_B]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteLCA[side]);

    // Create and initialize the spherical joint between upright and LCA.
    m_sphericalLCA[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLCA[side]->SetNameString(m_name + "_sphericalLCA" + suffix);
    m_sphericalLCA[side]->Initialize(m_LCA[side], m_upright[side], ChCoordsys<>(points[LCA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLCA[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_LCA[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_LCA[side], false, points[SPRING_C], points[SPRING_A], false,
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
double ChDoubleWishbone::GetMass() const {
    return 2 * (getSpindleMass() + getUCAMass() + getLCAMass() + getUprightMass());
}

// -----------------------------------------------------------------------------
// Get the current COM location of the suspension subsystem.
// -----------------------------------------------------------------------------
ChVector<> ChDoubleWishbone::GetCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += getSpindleMass() * m_spindle[LEFT]->GetPos();
    com += getSpindleMass() * m_spindle[RIGHT]->GetPos();

    com += getUCAMass() * m_UCA[LEFT]->GetPos();
    com += getUCAMass() * m_UCA[RIGHT]->GetPos();

    com += getLCAMass() * m_LCA[LEFT]->GetPos();
    com += getLCAMass() * m_LCA[RIGHT]->GetPos();

    com += getUprightMass() * m_upright[LEFT]->GetPos();
    com += getUprightMass() * m_upright[RIGHT]->GetPos();

    return com / GetMass();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChDoubleWishbone::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChDoubleWishbone::ReportSuspensionForce(VehicleSide side) const {
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
void ChDoubleWishbone::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteLCA[side]->GetC();
        GetLog() << "LCA revolute          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revoluteUCA[side]->GetC();
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

    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalLCA[side]->GetC();
        GetLog() << "LCA spherical         ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalUCA[side]->GetC();
        GetLog() << "UCA spherical         ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    // Distance constraint
    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for uprights
    AddVisualizationUpright(m_upright[LEFT], 0.5 * (m_pointsL[SPINDLE] + m_pointsL[UPRIGHT]), m_pointsL[UCA_U],
                            m_pointsL[LCA_U], m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], 0.5 * (m_pointsR[SPINDLE] + m_pointsR[UPRIGHT]), m_pointsR[UCA_U],
                            m_pointsR[LCA_U], m_pointsR[TIEROD_U], getUprightRadius());

    // Add visualization for upper control arms
    AddVisualizationControlArm(m_UCA[LEFT], m_pointsL[UCA_F], m_pointsL[UCA_B], m_pointsL[UCA_U], getUCARadius());
    AddVisualizationControlArm(m_UCA[RIGHT], m_pointsR[UCA_F], m_pointsR[UCA_B], m_pointsR[UCA_U], getUCARadius());

    // Add visualization for lower control arms
    AddVisualizationControlArm(m_LCA[LEFT], m_pointsL[LCA_F], m_pointsL[LCA_B], m_pointsL[LCA_U], getLCARadius());
    AddVisualizationControlArm(m_LCA[RIGHT], m_pointsR[LCA_F], m_pointsR[LCA_B], m_pointsR[LCA_U], getLCARadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(2 * getLCARadius(), 150, 15));
    m_spring[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(2 * getLCARadius(), 150, 15));

    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());

    // Add visualization for the tie-rods
    m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
    m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
}

void ChDoubleWishbone::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_UCA[LEFT]->GetAssets().clear();
    m_UCA[RIGHT]->GetAssets().clear();

    m_LCA[LEFT]->GetAssets().clear();
    m_LCA[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();

    m_distTierod[LEFT]->GetAssets().clear();
    m_distTierod[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
                                                  const ChVector<> pt_F,
                                                  const ChVector<> pt_B,
                                                  const ChVector<> pt_U,
                                                  double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_F = arm->TransformPointParentToLocal(pt_F);
    ChVector<> p_B = arm->TransformPointParentToLocal(pt_B);
    ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

    auto cyl_F = chrono_types::make_shared<ChCylinderShape>();
    cyl_F->GetCylinderGeometry().p1 = p_F;
    cyl_F->GetCylinderGeometry().p2 = p_U;
    cyl_F->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_F);

    auto cyl_B = chrono_types::make_shared<ChCylinderShape>();
    cyl_B->GetCylinderGeometry().p1 = p_B;
    cyl_B->GetCylinderGeometry().p2 = p_U;
    cyl_B->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_B);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.7f, 0.7f, 0.7f));
    arm->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                               const ChVector<> pt_C,
                                               const ChVector<> pt_U,
                                               const ChVector<> pt_L,
                                               const ChVector<> pt_T,
                                               double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_C = upright->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = upright->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);

    if ((p_L - p_C).Length2() > threshold2) {
        auto cyl_L = chrono_types::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = p_C;
        cyl_L->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_L);
    }

    if ((p_U - p_C).Length2() > threshold2) {
        auto cyl_U = chrono_types::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = p_C;
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_U);
    }

    if ((p_T - p_C).Length2() > threshold2) {
        auto cyl_T = chrono_types::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = p_C;
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_T);
    }

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_UCA[0]);
    bodies.push_back(m_UCA[1]);
    bodies.push_back(m_LCA[0]);
    bodies.push_back(m_LCA[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteUCA[0]);
    joints.push_back(m_revoluteUCA[1]);
    joints.push_back(m_sphericalUCA[0]);
    joints.push_back(m_sphericalUCA[1]);
    joints.push_back(m_revoluteLCA[0]);
    joints.push_back(m_revoluteLCA[1]);
    joints.push_back(m_sphericalLCA[0]);
    joints.push_back(m_sphericalLCA[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}


void ChDoubleWishbone::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_UCA[0]);
    bodies.push_back(m_UCA[1]);
    bodies.push_back(m_LCA[0]);
    bodies.push_back(m_LCA[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteUCA[0]);
    joints.push_back(m_revoluteUCA[1]);
    joints.push_back(m_sphericalUCA[0]);
    joints.push_back(m_sphericalUCA[1]);
    joints.push_back(m_revoluteLCA[0]);
    joints.push_back(m_revoluteLCA[1]);
    joints.push_back(m_sphericalLCA[0]);
    joints.push_back(m_sphericalLCA[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
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
