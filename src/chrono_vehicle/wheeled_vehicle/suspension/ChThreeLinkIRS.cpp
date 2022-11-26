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
// Base class for a 3-link independent rear suspension (non-steerable).
// This suspension has a trailing arm and 2 additional links connecting the
// arm to the chassis.
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
#include "chrono/assets/ChPointPointShape.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChThreeLinkIRS.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChThreeLinkIRS::m_pointNames[] = {"SPINDLE ", "TA_CM",    "TA_O",     "TA_I",    "TA_S",
                                                    "SHOCK_C ", "SHOCK_A ", "SPRING_C", "SPRING_A"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChThreeLinkIRS::ChThreeLinkIRS(const std::string& name) : ChSuspension(name) {}

ChThreeLinkIRS::~ChThreeLinkIRS() {
    auto sys = m_arm[0]->GetSystem();
    if (sys) {
        for (int i = 0; i < 2; i++) {
            sys->Remove(m_arm[i]);
            sys->Remove(m_upper[i]);
            sys->Remove(m_lower[i]);
            sys->Remove(m_sphericalArm[i]);
            sys->Remove(m_sphericalUpper[i]);
            sys->Remove(m_sphericalLower[i]);
            sys->Remove(m_universalUpper[i]);
            sys->Remove(m_universalLower[i]);
            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::Initialize(std::shared_ptr<ChChassis> chassis,
                                std::shared_ptr<ChSubchassis> subchassis,
                                std::shared_ptr<ChSteering> steering,
                                const ChVector<>& location,
                                double left_ang_vel,
                                double right_ang_vel) {
    m_parent = chassis;
    m_rel_loc = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform all hardpoints and directions to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);

    m_dirsL.resize(NUM_DIRS);
    m_dirsR.resize(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
        m_dirsL[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
        rel_dir.y() = -rel_dir.y();
        m_dirsR[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis->GetBody(), m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), m_pointsR, m_dirsR, right_ang_vel);
}

void ChThreeLinkIRS::InitializeSide(VehicleSide side,
                                    std::shared_ptr<ChBodyAuxRef> chassis,
                                    const std::vector<ChVector<> >& points,
                                    const std::vector<ChVector<>>& dirs,
                                    double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize the trailing arm and the two link bodies.
    u = points[TA_C] - points[TA_S];
    u.Normalize();
    v = Vcross(ChVector<>(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_arm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm[side]->SetNameString(m_name + "_arm" + suffix);
    m_arm[side]->SetPos(points[TA_CM]);
    m_arm[side]->SetRot(rot);
    m_arm[side]->SetMass(getArmMass());
    m_arm[side]->SetInertiaXX(getArmInertia());
    chassis->GetSystem()->AddBody(m_arm[side]);

    u = points[UL_A] - points[UL_C];
    u.Normalize();
    v = Vcross(ChVector<>(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_upper[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upper[side]->SetNameString(m_name + "_upper" + suffix);
    m_upper[side]->SetPos(points[UL_CM]);
    m_upper[side]->SetRot(rot);
    m_upper[side]->SetMass(getUpperLinkMass());
    m_upper[side]->SetInertiaXX(getUpperLinkInertia());
    chassis->GetSystem()->AddBody(m_upper[side]);

    u = points[LL_A] - points[LL_C];
    u.Normalize();
    v = Vcross(ChVector<>(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_lower[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lower[side]->SetNameString(m_name + "_lower" + suffix);
    m_lower[side]->SetPos(points[LL_CM]);
    m_lower[side]->SetRot(rot);
    m_lower[side]->SetMass(getLowerLinkMass());
    m_lower[side]->SetInertiaXX(getLowerLinkInertia());
    chassis->GetSystem()->AddBody(m_lower[side]);

    // Create and initialize the revolute joint between arm and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], spindleRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_arm[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spherical joint between chassis and arm.
    m_sphericalArm[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalArm[side]->SetNameString(m_name + "_sphericalArm" + suffix);
    m_sphericalArm[side]->Initialize(chassis, m_arm[side], ChCoordsys<>(points[TA_C], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalArm[side]);

    // Create and initialize the spherical joints between links and arm.
    m_sphericalUpper[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalUpper[side]->SetNameString(m_name + "_sphericalUpper" + suffix);
    m_sphericalUpper[side]->Initialize(m_upper[side], m_arm[side], ChCoordsys<>(points[UL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUpper[side]);

    m_sphericalLower[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLower[side]->SetNameString(m_name + "_sphericalLower" + suffix);
    m_sphericalLower[side]->Initialize(m_lower[side], m_arm[side], ChCoordsys<>(points[LL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLower[side]);

    // Create and initialize the universal joints between links and chassis.
    u = dirs[UNIV_AXIS_UPPER];
    w = Vcross(u, ChVector<>(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_universalUpper[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalUpper[side]->SetNameString(m_name + "_universalUpper" + suffix);
    m_universalUpper[side]->Initialize(m_upper[side], chassis, ChFrame<>(points[UL_C], rot));
    chassis->GetSystem()->AddLink(m_universalUpper[side]);

    u = dirs[UNIV_AXIS_LOWER];
    w = Vcross(u, ChVector<>(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_universalLower[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalLower[side]->SetNameString(m_name + "_universalLower" + suffix);
    m_universalLower[side]->Initialize(m_lower[side], chassis, ChFrame<>(points[LL_C], rot));
    chassis->GetSystem()->AddLink(m_universalLower[side]);

    // Create and initialize the spring/damper.
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_arm[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_arm[side], false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}


void ChThreeLinkIRS::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getArmMass() + getLowerLinkMass() + getUpperLinkMass());
}

void ChThreeLinkIRS::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaArm(getArmInertia());
    ChMatrix33<> inertiaLower(getLowerLinkInertia());
    ChMatrix33<> inertiaUpper(getUpperLinkInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_arm[LEFT]->GetFrame_COG_to_abs(), getArmMass(), inertiaArm);
    composite.AddComponent(m_arm[RIGHT]->GetFrame_COG_to_abs(), getArmMass(), inertiaArm);
    composite.AddComponent(m_lower[LEFT]->GetFrame_COG_to_abs(), getLowerLinkMass(), inertiaLower);
    composite.AddComponent(m_lower[RIGHT]->GetFrame_COG_to_abs(), getLowerLinkMass(), inertiaLower);
    composite.AddComponent(m_upper[LEFT]->GetFrame_COG_to_abs(), getUpperLinkMass(), inertiaUpper);
    composite.AddComponent(m_upper[RIGHT]->GetFrame_COG_to_abs(), getUpperLinkMass(), inertiaUpper);

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChThreeLinkIRS::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChThreeLinkIRS::ReportSuspensionForce(VehicleSide side) const {
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
void ChThreeLinkIRS::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::LogConstraintViolations(VehicleSide side) {
    {
        ChVectorDynamic<> C = m_sphericalArm[side]->GetConstraintViolation();
        GetLog() << "Arm spherical         ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalUpper[side]->GetConstraintViolation();
        GetLog() << "Upper spherical       ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLower[side]->GetConstraintViolation();
        GetLog() << "Lower spherical       ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalUpper[side]->GetConstraintViolation();
        GetLog() << "Upper universal       ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalLower[side]->GetConstraintViolation();
        GetLog() << "Lower universal       ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
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
void ChThreeLinkIRS::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for trailing arms
    AddVisualizationArm(m_arm[LEFT], m_pointsL[TA_C], m_pointsL[TA_S], m_pointsL[TA_CM], m_pointsL[UL_A],
                        m_pointsL[LL_A], getArmRadius());
    AddVisualizationArm(m_arm[RIGHT], m_pointsR[TA_C], m_pointsR[TA_S], m_pointsR[TA_CM], m_pointsR[UL_A],
                        m_pointsR[LL_A], getArmRadius());

    // Add visualization for upper links
    AddVisualizationLink(m_upper[LEFT], m_pointsL[UL_C], m_pointsL[UL_A], m_pointsL[UL_CM], getUpperLinkRadius());
    AddVisualizationLink(m_upper[RIGHT], m_pointsR[UL_C], m_pointsR[UL_A], m_pointsR[UL_CM], getUpperLinkRadius());

    // Add visualization for lower links
    AddVisualizationLink(m_lower[LEFT], m_pointsL[LL_C], m_pointsL[LL_A], m_pointsL[LL_CM], getLowerLinkRadius());
    AddVisualizationLink(m_lower[RIGHT], m_pointsR[LL_C], m_pointsR[LL_A], m_pointsR[LL_CM], getLowerLinkRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
}

void ChThreeLinkIRS::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_arm[LEFT]);
    ChPart::RemoveVisualizationAssets(m_arm[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_upper[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upper[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_lower[LEFT]);
    ChPart::RemoveVisualizationAssets(m_lower[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::AddVisualizationArm(std::shared_ptr<ChBody> body,
    const ChVector<>& pt_C,
    const ChVector<>& pt_S,
    const ChVector<>& pt_CM,
    const ChVector<>& pt_U,
    const ChVector<>& pt_L,
    double radius)
{
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_C = body->TransformPointParentToLocal(pt_C);
    ChVector<> p_S = body->TransformPointParentToLocal(pt_S);
    ChVector<> p_CM = body->TransformPointParentToLocal(pt_CM);
    ChVector<> p_U = body->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = body->TransformPointParentToLocal(pt_L);

    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_C;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_S;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_2);

    if ((p_S - p_U).Length2() > threshold2) {
        auto cyl_U = chrono_types::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_S;
        cyl_U->GetCylinderGeometry().p2 = p_U;
        cyl_U->GetCylinderGeometry().rad = radius;
        body->AddVisualShape(cyl_U);
    }

    if ((p_S - p_L).Length2() > threshold2) {
        auto cyl_L = chrono_types::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_S;
        cyl_L->GetCylinderGeometry().p2 = p_L;
        cyl_L->GetCylinderGeometry().rad = radius;
        body->AddVisualShape(cyl_L);
    }
}

void ChThreeLinkIRS::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                          const ChVector<>& pt_1,
                                          const ChVector<>& pt_2,
                                          const ChVector<>& pt_CM,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);
    ChVector<> p_CM = body->TransformPointParentToLocal(pt_CM);

    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_1;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_2;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_2);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_upper[0]);
    bodies.push_back(m_upper[1]);
    bodies.push_back(m_lower[0]);
    bodies.push_back(m_lower[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalArm[0]);
    joints.push_back(m_sphericalArm[1]);
    joints.push_back(m_sphericalUpper[0]);
    joints.push_back(m_sphericalUpper[1]);
    joints.push_back(m_sphericalLower[0]);
    joints.push_back(m_sphericalLower[1]);
    joints.push_back(m_universalUpper[0]);
    joints.push_back(m_universalUpper[1]);
    joints.push_back(m_universalLower[0]);
    joints.push_back(m_universalLower[1]);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChThreeLinkIRS::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_upper[0]);
    bodies.push_back(m_upper[1]);
    bodies.push_back(m_lower[0]);
    bodies.push_back(m_lower[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalArm[0]);
    joints.push_back(m_sphericalArm[1]);
    joints.push_back(m_sphericalUpper[0]);
    joints.push_back(m_sphericalUpper[1]);
    joints.push_back(m_sphericalLower[0]);
    joints.push_back(m_sphericalLower[1]);
    joints.push_back(m_universalUpper[0]);
    joints.push_back(m_universalUpper[1]);
    joints.push_back(m_universalLower[0]);
    joints.push_back(m_universalLower[1]);
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
