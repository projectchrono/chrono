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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Base class for a multi-link suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMultiLink::m_pointNames[] = {"SPINDLE  ",
                                                 "UPRIGHT  ",
                                                 "UA_F     ",
                                                 "UA_B     ",
                                                 "UA_U     ",
                                                 "UA_CM    ",
                                                 "LAT_C    ",
                                                 "LAT_U    ",
                                                 "LAT_CM   ",
                                                 "TL_C     ",
                                                 "TL_U     ",
                                                 "TL_CM    ",
                                                 "SHOCK_C  ",
                                                 "SHOCK_L  ",
                                                 "SPRING_C ",
                                                 "SPRING_L ",
                                                 "TIEROD_C ",
                                                 "TIEROD_U "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMultiLink::ChMultiLink(const std::string& name) : ChSuspension(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                             const ChVector<>& location,
                             std::shared_ptr<ChBody> tierod_body,
                             double left_ang_vel,
                             double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame
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
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, m_dirsR, right_ang_vel);
}

void ChMultiLink::InitializeSide(VehicleSide side,
                                 std::shared_ptr<ChBodyAuxRef> chassis,
                                 std::shared_ptr<ChBody> tierod_body,
                                 const std::vector<ChVector<> >& points,
                                 const std::vector<ChVector<> >& dirs,
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
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize Upper Arm body.
    // Determine the rotation matrix of the upper arm based on the plane of the hard points
    // (z axis normal to the plane of the upper arm)
    w = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
    w.Normalize();
    u = points[UA_F] - points[UA_B];
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_upperArm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upperArm[side]->SetNameString(m_name + "_upperArm" + suffix);
    m_upperArm[side]->SetPos(points[UA_CM]);
    m_upperArm[side]->SetRot(rot);
    m_upperArm[side]->SetMass(getUpperArmMass());
    m_upperArm[side]->SetInertiaXX(getUpperArmInertia());
    chassis->GetSystem()->AddBody(m_upperArm[side]);

    // Create and initialize lateral body.
    // Determine the rotation matrix of the lateral based on the plane of the hard points
    // (z-axis along the length of the track rod)
    v = Vcross(points[LAT_U] - points[TL_U], points[LAT_C] - points[TL_U]);
    v.Normalize();
    w = points[LAT_C] - points[LAT_U];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_lateral[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lateral[side]->SetNameString(m_name + "_lateral" + suffix);
    m_lateral[side]->SetPos(points[LAT_CM]);
    m_lateral[side]->SetRot(rot);
    m_lateral[side]->SetMass(getLateralMass());
    m_lateral[side]->SetInertiaXX(getLateralInertia());
    chassis->GetSystem()->AddBody(m_lateral[side]);

    // Create and initialize trailing link body.
    // Determine the rotation matrix of the trailing link based on the plane of the hard points
    // (z-axis along the length of the trailing link)
    v = Vcross(points[TL_U] - points[SPRING_L], points[TL_C] - points[SPRING_L]);
    v.Normalize();
    w = points[TL_C] - points[TL_U];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_trailingLink[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_trailingLink[side]->SetNameString(m_name + "_trailingLink" + suffix);
    m_trailingLink[side]->SetPos(points[TL_CM]);
    m_trailingLink[side]->SetRot(rot);
    m_trailingLink[side]->SetMass(getTrailingLinkMass());
    m_trailingLink[side]->SetInertiaXX(getTrailingLinkInertia());
    chassis->GetSystem()->AddBody(m_trailingLink[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and upper arm.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the upper arm.
    v = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
    v.Normalize();
    w = points[UA_F] - points[UA_B];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_revoluteUA[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteUA[side]->SetNameString(m_name + "_revoluteUA" + suffix);
    m_revoluteUA[side]->Initialize(chassis, m_upperArm[side],
                                   ChCoordsys<>((points[UA_F] + points[UA_B]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteUA[side]);

    // Create and initialize the spherical joint between upright and upper arm.
    m_sphericalUA[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalUA[side]->SetNameString(m_name + "_sphericalUA" + suffix);
    m_sphericalUA[side]->Initialize(m_upperArm[side], m_upright[side], ChCoordsys<>(points[UA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUA[side]);

    // Create and initialize the spherical joint between upright and track rod.
    m_sphericalLateralUpright[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalLateralUpright[side]->SetNameString(m_name + "_sphericalLateralUpright" + suffix);
    m_sphericalLateralUpright[side]->Initialize(m_lateral[side], m_upright[side], ChCoordsys<>(points[LAT_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLateralUpright[side]);

    // Create and initialize the universal joint between chassis and track rod.
    u = dirs[UNIV_AXIS_CHASSIS_LAT];
    v = dirs[UNIV_AXIS_LINK_LAT];
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_universalLateralChassis[side] = std::make_shared<ChLinkUniversal>();
    m_universalLateralChassis[side]->SetNameString(m_name + "_universalLateralChassis" + suffix);
    m_universalLateralChassis[side]->Initialize(m_lateral[side], chassis,
                                                ChFrame<>(points[LAT_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalLateralChassis[side]);

    // Create and initialize the spherical joint between upright and trailing link.
    m_sphericalTLUpright[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalTLUpright[side]->SetNameString(m_name + "_sphericalTLUpright" + suffix);
    m_sphericalTLUpright[side]->Initialize(m_trailingLink[side], m_upright[side], ChCoordsys<>(points[TL_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTLUpright[side]);

    // Create and initialize the universal joint between chassis and trailing link.
    u = dirs[UNIV_AXIS_CHASSIS_TL];
    v = dirs[UNIV_AXIS_LINK_TL];
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_universalTLChassis[side] = std::make_shared<ChLinkUniversal>();
    m_universalTLChassis[side]->SetNameString(m_name + "_universalTLChassis" + suffix);
    m_universalTLChassis[side]->Initialize(m_trailingLink[side], chassis,
                                           ChFrame<>(points[TL_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalTLChassis[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    m_distTierod[side] = std::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring/damper
    m_shock[side] = std::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_trailingLink[side], false, points[SHOCK_C], points[SHOCK_L]);
    m_shock[side]->Set_SpringCallback(getShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = std::make_shared<ChLinkSpringCB>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_trailingLink[side], false, points[SPRING_C], points[SPRING_L], false,
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
double ChMultiLink::GetMass() const {
    return 2 * (getSpindleMass() + getUpperArmMass() + getLateralMass() + getTrailingLinkMass() + getUprightMass());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChMatrix<>* C = m_revoluteUA[side]->GetC();
        GetLog() << "Upper arm revolute    ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
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

    // Spherical joints
    {
        ChMatrix<>* C = m_sphericalUA[side]->GetC();
        GetLog() << "Upper arm spherical   ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalLateralUpright[side]->GetC();
        GetLog() << "Lateral-Upright spherical  ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalTLUpright[side]->GetC();
        GetLog() << "TL-Upright spherical  ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }

    // Universal joints
    {
        ChMatrix<>* C = m_universalLateralChassis[side]->GetC();
        GetLog() << "Lateral-Chassis universal  ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_universalTLChassis[side]->GetC();
        GetLog() << "TL-Chassis universal  ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }

    // Distance constraint
    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationUpright(m_upright[LEFT], m_pointsL[UA_U], m_pointsL[LAT_U], m_pointsL[TL_U], m_pointsL[TIEROD_U],
                            m_pointsL[UPRIGHT], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], m_pointsR[UA_U], m_pointsR[LAT_U], m_pointsR[TL_U], m_pointsR[TIEROD_U],
                            m_pointsR[UPRIGHT], getUprightRadius());

    AddVisualizationUpperArm(m_upperArm[LEFT], m_pointsL[UA_F], m_pointsL[UA_B], m_pointsL[UA_U], getUpperArmRadius());
    AddVisualizationUpperArm(m_upperArm[RIGHT], m_pointsR[UA_F], m_pointsR[UA_B], m_pointsR[UA_U], getUpperArmRadius());

    AddVisualizationLateral(m_lateral[LEFT], m_pointsL[LAT_U], m_pointsL[LAT_C], getLateralRadius());
    AddVisualizationLateral(m_lateral[RIGHT], m_pointsR[LAT_U], m_pointsR[LAT_C], getLateralRadius());

    AddVisualizationTrailingLink(m_trailingLink[LEFT], m_pointsL[TL_C], m_pointsL[SPRING_L], m_pointsL[TL_U],
                                 getTrailingLinkRadius());
    AddVisualizationTrailingLink(m_trailingLink[RIGHT], m_pointsR[TL_C], m_pointsR[SPRING_L], m_pointsR[TL_U],
                                 getTrailingLinkRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());

    // Add visualization for the tie-rods
    m_distTierod[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[LEFT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
}

void ChMultiLink::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_upperArm[LEFT]->GetAssets().clear();
    m_upperArm[RIGHT]->GetAssets().clear();

    m_lateral[LEFT]->GetAssets().clear();
    m_lateral[RIGHT]->GetAssets().clear();

    m_trailingLink[LEFT]->GetAssets().clear();
    m_trailingLink[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();

    m_distTierod[LEFT]->GetAssets().clear();
    m_distTierod[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::AddVisualizationUpperArm(std::shared_ptr<ChBody> arm,
                                           const ChVector<> pt_F,
                                           const ChVector<> pt_B,
                                           const ChVector<> pt_U,
                                           double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_F = arm->TransformPointParentToLocal(pt_F);
    ChVector<> p_B = arm->TransformPointParentToLocal(pt_B);
    ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

    auto cyl_F = std::make_shared<ChCylinderShape>();
    cyl_F->GetCylinderGeometry().p1 = p_F;
    cyl_F->GetCylinderGeometry().p2 = p_U;
    cyl_F->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_F);

    auto cyl_B = std::make_shared<ChCylinderShape>();
    cyl_B->GetCylinderGeometry().p1 = p_B;
    cyl_B->GetCylinderGeometry().p2 = p_U;
    cyl_B->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_B);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.6f, 0.2f, 0.6f));
    arm->AddAsset(col);
}

void ChMultiLink::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                          const ChVector<> pt_UA,
                                          const ChVector<> pt_TR,
                                          const ChVector<> pt_TL,
                                          const ChVector<> pt_T,
                                          const ChVector<> pt_U,
                                          double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_UA = upright->TransformPointParentToLocal(pt_UA);
    ChVector<> p_TR = upright->TransformPointParentToLocal(pt_TR);
    ChVector<> p_TL = upright->TransformPointParentToLocal(pt_TL);
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);

    if (p_UA.Length2() > threshold2) {
        auto cyl_UA = std::make_shared<ChCylinderShape>();
        cyl_UA->GetCylinderGeometry().p1 = p_UA;
        cyl_UA->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_UA->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_UA);
    }

    if (p_TR.Length2() > threshold2) {
        auto cyl_TR= std::make_shared<ChCylinderShape>();
        cyl_TR->GetCylinderGeometry().p1 = p_TR;
        cyl_TR->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_TR->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_TR);
    }

    if (p_TL.Length2() > threshold2) {
        auto cyl_TL = std::make_shared<ChCylinderShape>();
        cyl_TL->GetCylinderGeometry().p1 = p_TL;
        cyl_TL->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_TL->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_TL);
    }

    if (p_T.Length2() > threshold2) {
        auto cyl_T = std::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_T);
    }

    if (p_U.Length2() > threshold2) {
        auto cyl_U = std::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_U);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

void ChMultiLink::AddVisualizationLateral(std::shared_ptr<ChBody> rod,
                                          const ChVector<> pt_C,
                                          const ChVector<> pt_U,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = rod->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = rod->TransformPointParentToLocal(pt_U);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_C;
    cyl->GetCylinderGeometry().p2 = p_U;
    cyl->GetCylinderGeometry().rad = radius;
    rod->AddAsset(cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.2f));
    rod->AddAsset(col);
}

void ChMultiLink::AddVisualizationTrailingLink(std::shared_ptr<ChBody> link,
                                               const ChVector<> pt_C,
                                               const ChVector<> pt_S,
                                               const ChVector<> pt_U,
                                               double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = link->TransformPointParentToLocal(pt_C);
    ChVector<> p_S = link->TransformPointParentToLocal(pt_S);
    ChVector<> p_U = link->TransformPointParentToLocal(pt_U);

    auto cyl1 = std::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().p1 = p_C;
    cyl1->GetCylinderGeometry().p2 = p_S;
    cyl1->GetCylinderGeometry().rad = radius;
    link->AddAsset(cyl1);

    auto cyl2 = std::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().p1 = p_S;
    cyl2->GetCylinderGeometry().p2 = p_U;
    cyl2->GetCylinderGeometry().rad = radius;
    link->AddAsset(cyl2);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.6f));
    link->AddAsset(col);
}

}  // end namespace vehicle
}  // end namespace chrono
