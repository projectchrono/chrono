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
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                const ChVector<>& location,
                                std::shared_ptr<ChBody> tierod_body,
                                double left_ang_vel,
                                double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

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
    InitializeSide(LEFT, chassis, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, m_dirsR, right_ang_vel);
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
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_arm[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spherical joint between chassis and arm.
    m_sphericalArm[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalArm[side]->SetNameString(m_name + "_sphericalArm" + suffix);
    m_sphericalArm[side]->Initialize(chassis, m_arm[side], ChCoordsys<>(points[TA_C], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalArm[side]);

    // Create and initialize the spherical joints between links and arm.
    m_sphericalUpper[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalUpper[side]->SetNameString(m_name + "_sphericalUpper" + suffix);
    m_sphericalUpper[side]->Initialize(m_upper[side], m_arm[side], ChCoordsys<>(points[UL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUpper[side]);

    m_sphericalLower[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalLower[side]->SetNameString(m_name + "_sphericalLower" + suffix);
    m_sphericalLower[side]->Initialize(m_lower[side], m_arm[side], ChCoordsys<>(points[LL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLower[side]);

    // Create and initialize the universal joints between links and chassis.
    u = dirs[UNIV_AXIS_UPPER];
    w = Vcross(u, ChVector<>(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_universalUpper[side] = std::make_shared<ChLinkUniversal>();
    m_universalUpper[side]->SetNameString(m_name + "_universalUpper" + suffix);
    m_universalUpper[side]->Initialize(m_upper[side], chassis, ChFrame<>(points[UL_C], rot));
    chassis->GetSystem()->AddLink(m_universalUpper[side]);

    u = dirs[UNIV_AXIS_LOWER];
    w = Vcross(u, ChVector<>(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_universalLower[side] = std::make_shared<ChLinkUniversal>();
    m_universalLower[side]->SetNameString(m_name + "_universalLower" + suffix);
    m_universalLower[side]->Initialize(m_lower[side], chassis, ChFrame<>(points[LL_C], rot));
    chassis->GetSystem()->AddLink(m_universalLower[side]);

    // Create and initialize the spring/damper.
    m_shock[side] = std::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_arm[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->Set_SpringCallback(getShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = std::make_shared<ChLinkSpringCB>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_arm[side], false, points[SPRING_C], points[SPRING_A], false,
                               getSpringRestLength());
    m_spring[side]->Set_SpringCallback(getSpringForceCallback());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = std::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side] = std::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

// -----------------------------------------------------------------------------
// Get the total mass of the suspension subsystem.
// -----------------------------------------------------------------------------
double ChThreeLinkIRS::GetMass() const {
    return 2 * (getSpindleMass() + getArmMass() + getLowerLinkMass() + getUpperLinkMass());
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
        ChMatrix<>* C = m_sphericalArm[side]->GetC();
        GetLog() << "Arm spherical         ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalUpper[side]->GetC();
        GetLog() << "Upper spherical       ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalLower[side]->GetC();
        GetLog() << "Lower spherical       ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_universalUpper[side]->GetC();
        GetLog() << "Upper universal       ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_universalLower[side]->GetC();
        GetLog() << "Lower universal       ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_revolute[side]->GetC();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
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
    m_spring[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
}

void ChThreeLinkIRS::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_arm[LEFT]->GetAssets().clear();
    m_arm[RIGHT]->GetAssets().clear();

    m_upper[LEFT]->GetAssets().clear();
    m_upper[RIGHT]->GetAssets().clear();

    m_lower[LEFT]->GetAssets().clear();
    m_lower[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();
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

    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_C;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_S;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_2);

    if ((p_S - p_U).Length2() > threshold2) {
        auto cyl_U = std::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_S;
        cyl_U->GetCylinderGeometry().p2 = p_U;
        cyl_U->GetCylinderGeometry().rad = radius;
        body->AddAsset(cyl_U);
    }

    if ((p_S - p_L).Length2() > threshold2) {
        auto cyl_L = std::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_S;
        cyl_L->GetCylinderGeometry().p2 = p_L;
        cyl_L->GetCylinderGeometry().rad = radius;
        body->AddAsset(cyl_L);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.8f, 0.2f));
    body->AddAsset(col);
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

    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_1;
    cyl_1->GetCylinderGeometry().p2 = p_CM;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_2;
    cyl_2->GetCylinderGeometry().p2 = p_CM;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_2);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.8f, 0.2f, 0.2f));
    body->AddAsset(col);
}

}  // end namespace vehicle
}  // end namespace chrono
