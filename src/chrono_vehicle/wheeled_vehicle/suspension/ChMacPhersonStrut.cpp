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
// Authors: Daniel Melanz
// =============================================================================
//
// Base class for a MacPherson strut modeled with bodies and constraints.
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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMacPhersonStrut::m_pointNames[] = {"SPINDLE ",
                                                      "UPRIGHT ",
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
ChMacPhersonStrut::ChMacPhersonStrut(const std::string& name) : ChSuspension(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                   const ChVector<>& location,
                                   std::shared_ptr<ChBody> tierod_body,
                                   double left_ang_vel,
                                   double right_ang_vel) {
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
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, right_ang_vel);
}

void ChMacPhersonStrut::InitializeSide(VehicleSide side,
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
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize Strut body.
    m_strut[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_strut[side]->SetNameString(m_name + "_Strut" + suffix);
    m_strut[side]->SetPos((points[SPRING_C]+points[SPRING_U])/2);
    m_strut[side]->SetRot(chassisRot);
    m_strut[side]->SetMass(getStrutMass());
    m_strut[side]->SetInertiaXX(getStrutInertia());
    chassis->GetSystem()->AddBody(m_strut[side]);

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
    m_LCA[side]->SetInertiaXX(getLCAInertia());
    chassis->GetSystem()->AddBody(m_LCA[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(-CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the cylindrical joint between upright and strut.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the UCA.
    w = points[SPRING_C] - points[SPRING_U];
    w.Normalize();
    v = Vcross(points[SPRING_U] - points[LCA_B], points[SPRING_C] - points[LCA_B]);
    v.Normalize();
    u = Vcross(v, w);
    u.Normalize();
    rot.Set_A_axis(u, v, w);

    m_cylindricalStrut[side] = std::make_shared<ChLinkLockCylindrical>();
    m_cylindricalStrut[side]->SetNameString(m_name + "_cylindricalStrut" + suffix);
    m_cylindricalStrut[side]->Initialize(m_strut[side], m_upright[side], ChCoordsys<>(points[SPRING_U], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_cylindricalStrut[side]);

    // Create and initialize the universal joint between chassis and UCA.
    //// Determine the joint orientation matrix from the hardpoint locations by
    //// constructing a rotation matrix with the z axis along the joint direction
    //// and the y axis normal to the plane of the UCA.
    //v = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
    //v.Normalize();
    //w = points[UCA_F] - points[UCA_B];
    //w.Normalize();
    //u = Vcross(v, w);
    //rot.Set_A_axis(u, v, w);
    //TODO: Is this the correct rotation matrix?
    m_universalStrut[side] = std::make_shared<ChLinkUniversal>();
    m_universalStrut[side]->SetNameString(m_name + "_universalStrut" + suffix);
    m_universalStrut[side]->Initialize(chassis, m_strut[side],
      ChFrame<>(points[SPRING_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalStrut[side]);

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

    m_revoluteLCA[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteLCA[side]->SetNameString(m_name + "_revoluteLCA" + suffix);
    m_revoluteLCA[side]->Initialize(chassis, m_LCA[side],
                                    ChCoordsys<>((points[LCA_F] + points[LCA_B]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteLCA[side]);

    // Create and initialize the spherical joint between upright and LCA.
    m_sphericalLCA[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalLCA[side]->SetNameString(m_name + "_sphericalLCA" + suffix);
    m_sphericalLCA[side]->Initialize(m_LCA[side], m_upright[side], ChCoordsys<>(points[LCA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLCA[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    m_distTierod[side] = std::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring/damper
    m_shock[side] = std::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
    m_shock[side]->Set_SpringCallback(getShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = std::make_shared<ChLinkSpringCB>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_upright[side], false, points[SPRING_C], points[SPRING_U], false,
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
double ChMacPhersonStrut::GetMass() const {
    return 2 * (getSpindleMass() + getStrutMass() + getLCAMass() + getUprightMass());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChMatrix<>* C = m_revoluteLCA[side]->GetC();
        GetLog() << "LCA revolute          ";
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
        ChMatrix<>* C = m_sphericalLCA[side]->GetC();
        GetLog() << "LCA spherical         ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }

    // Universal joints
    {
      ChMatrix<>* C = m_universalStrut[side]->GetC();
      GetLog() << "Strut universal       ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "\n";
      GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }

  // Cylindrical joints
    {
      ChMatrix<>* C = m_cylindricalStrut[side]->GetC();
      GetLog() << "Strut cylindrical     ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "\n";
      GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }

    // Distance constraint
    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationUpright(m_upright[LEFT], 0.5 * (m_pointsL[SPINDLE] + m_pointsL[UPRIGHT]), m_pointsL[SPRING_U], m_pointsL[LCA_U],
        m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], 0.5 * (m_pointsR[SPINDLE] + m_pointsR[UPRIGHT]), m_pointsR[SPRING_U], m_pointsR[LCA_U],
        m_pointsR[TIEROD_U], getUprightRadius());

    AddVisualizationStrut(m_strut[LEFT], m_pointsL[SPRING_C], m_pointsL[SPRING_U], getStrutRadius());
    AddVisualizationStrut(m_strut[RIGHT], m_pointsR[SPRING_C], m_pointsR[SPRING_U], getStrutRadius());

    AddVisualizationControlArm(m_LCA[LEFT], m_pointsL[LCA_F], m_pointsL[LCA_B], m_pointsL[LCA_U], getLCARadius());
    AddVisualizationControlArm(m_LCA[RIGHT], m_pointsR[LCA_F], m_pointsR[LCA_B], m_pointsR[LCA_U], getLCARadius());

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

void ChMacPhersonStrut::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_strut[LEFT]->GetAssets().clear();
    m_strut[RIGHT]->GetAssets().clear();

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
void ChMacPhersonStrut::AddVisualizationStrut(std::shared_ptr<ChBody> strut, 
                                              const ChVector<> pt_c, 
                                              const ChVector<> pt_u,
                                              double radius) {
  // Express hardpoint locations in body frame.
  ChVector<> p_c = strut->TransformPointParentToLocal(pt_c);
  ChVector<> p_u = strut->TransformPointParentToLocal(pt_u);

  auto cyl = std::make_shared<ChCylinderShape>();
  cyl->GetCylinderGeometry().p1 = p_c;
  cyl->GetCylinderGeometry().p2 = p_u;
  cyl->GetCylinderGeometry().rad = radius;
  strut->AddAsset(cyl);

  auto col = std::make_shared<ChColorAsset>();
  col->SetColor(ChColor(0.7f, 0.7f, 0.7f));
  strut->AddAsset(col);
}

void ChMacPhersonStrut::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
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
    col->SetColor(ChColor(0.7f, 0.7f, 0.7f));
    arm->AddAsset(col);
}

void ChMacPhersonStrut::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
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
        auto cyl_L = std::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = p_C;
        cyl_L->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_L);
    }

    if ((p_U - p_C).Length2() > threshold2) {
        auto cyl_U = std::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = p_C;
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_U);
    }

    if ((p_T - p_C).Length2() > threshold2) {
        auto cyl_T = std::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = p_C;
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_T);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

}  // end namespace vehicle
}  // end namespace chrono
