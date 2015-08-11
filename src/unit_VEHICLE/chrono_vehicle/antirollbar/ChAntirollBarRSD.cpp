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
// Base class for an anti-roll bar template modeled two arms connected with a
// revolute spring-damper.
// Derived from ChAntirollBar, but still and abstract base class.
//
// The anti-roll bar subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The subsystem reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "chrono_vehicle/antirollbar/ChAntirollBarRSD.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChAntirollBarRSD::ChAntirollBarRSD(const std::string& name)
: ChAntirollBar(name)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                                  const ChVector<>&          location,
                                  ChSharedPtr<ChBody>        susp_body_left,
                                  ChSharedPtr<ChBody>        susp_body_right)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> subsystem_to_abs(location);
  subsystem_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Chassis orientation (expressed in absolute frame)
  // Recall that the subsystem reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Convenience names
  double L = getArmLength();
  double W = getArmWidth();
  double H = getDroplinkHeight();

  // Express the local coordinates into the absolute coordinate system
  ChVector<> P_center = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, 0, 0));
  ChVector<> P_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, L / 2, 0));
  ChVector<> P_drop_arm_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, L, 0));
  ChVector<> P_drop_susp_left = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, L, H));
  ChVector<> P_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(0, -L / 2, 0));
  ChVector<> P_drop_arm_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, -L, 0));
  ChVector<> P_drop_susp_right = subsystem_to_abs.TransformPointLocalToParent(ChVector<>(W, -L, H));

  // Create an initialize the arm_left body
  m_arm_left = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_arm_left->SetNameString(m_name + "_arm_left");
  m_arm_left->SetPos(P_arm_left);
  m_arm_left->SetRot(subsystem_to_abs.GetRot());
  m_arm_left->SetMass(getArmMass());			
  m_arm_left->SetInertiaXX(getArmInertia());
  AddVisualizationArm(m_arm_left, ChVector<>(0, -L / 2, 0), ChVector<>(0, L / 2, 0), ChVector<>(W, L / 2, 0), getArmRadius(), ChColor(0.7f, 0.2f, 0.2f));
  chassis->GetSystem()->AddBody(m_arm_left);

  // Create an initialize the arm_right body
  m_arm_right = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_arm_right->SetNameString(m_name + "_arm_right");
  m_arm_right->SetPos(P_arm_right);
  m_arm_right->SetRot(subsystem_to_abs.GetRot());
  m_arm_right->SetMass(getArmMass());
  m_arm_right->SetInertiaXX(getArmInertia());
  AddVisualizationArm(m_arm_right, ChVector<>(0, L / 2, 0), ChVector<>(0, -L / 2, 0), ChVector<>(W, -L / 2, 0), getArmRadius(), ChColor(0.2f, 0.7f, 0.2f));
  chassis->GetSystem()->AddBody(m_arm_right);

  // Create and initialize the revolute joint between left arm and chassis.
  ChCoordsys<> rev_ch_csys(P_arm_left, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute_ch = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute_ch->SetNameString(m_name + "_revolute_ch");
  m_revolute_ch->Initialize(m_arm_left, chassis, rev_ch_csys);
  chassis->GetSystem()->AddLink(m_revolute_ch);

  // Create and initialize the revolute joint between left and right arms.
  ChCoordsys<> rev_csys(P_center, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute->SetNameString(m_name + "_revolute");
  m_revolute->Initialize(m_arm_left, m_arm_right, rev_csys);
  chassis->GetSystem()->AddLink(m_revolute);

  ChLinkForce* RSD = new ChLinkForce();
  RSD->Set_active(1);
  RSD->Set_K(getSpringCoefficient());
  RSD->Set_R(getDampingCoefficient());
  m_revolute->SetForce_Rz(RSD);

  // Create distance constraint to model left droplink.
  m_link_left = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_link_left->SetNameString(m_name + "_droplink_left");
  m_link_left->Initialize(m_arm_left, susp_body_left, false, P_drop_arm_left, P_drop_susp_left);
  chassis->GetSystem()->AddLink(m_link_left);

  // Create distance constraint to model right droplink.
  m_link_right = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_link_right->SetNameString(m_name + "_droplink_right");
  m_link_right->Initialize(m_arm_right, susp_body_right, false, P_drop_arm_right, P_drop_susp_right);
  chassis->GetSystem()->AddLink(m_link_right);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::LogConstraintViolations()
{
  // Chassis revolute joint
  {
    ChMatrix<> *C = m_revolute_ch->GetC();
    GetLog() << "Chassis revolute          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Central revolute joint
  {
    ChMatrix<> *C = m_revolute->GetC();
    GetLog() << "Central revolute          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Distance constraints (droplinks)
  GetLog() << "Droplink distance (left)  ";
  GetLog() << "  " << m_link_left->GetCurrentDistance() - m_link_left->GetImposedDistance() << "\n";
  GetLog() << "Droplink distance (right) ";
  GetLog() << "  " << m_link_right->GetCurrentDistance() - m_link_right->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChAntirollBarRSD::AddVisualizationArm(ChSharedPtr<ChBody>  arm,
                                           const ChVector<>&    pt_1,
                                           const ChVector<>&    pt_2,
                                           const ChVector<>&    pt_3,
                                           double               radius,
                                           const ChColor&       color)
{
  ChSharedPtr<ChCylinderShape> cyl_1(new ChCylinderShape);
  cyl_1->GetCylinderGeometry().p1 = pt_1;
  cyl_1->GetCylinderGeometry().p2 = pt_2;
  cyl_1->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_1);

  ChSharedPtr<ChCylinderShape> cyl_2(new ChCylinderShape);
  cyl_2->GetCylinderGeometry().p1 = pt_2;
  cyl_2->GetCylinderGeometry().p2 = pt_3;
  cyl_2->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_2);

  ChSharedPtr<ChCylinderShape> cyl_3(new ChCylinderShape);
  cyl_3->GetCylinderGeometry().p1 = pt_1 + ChVector<>(0, 0, 3 * radius);
  cyl_3->GetCylinderGeometry().p2 = pt_1 - ChVector<>(0, 0, 3 * radius);
  cyl_3->GetCylinderGeometry().rad = radius / 2;
  arm->AddAsset(cyl_3);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(color);
  arm->AddAsset(col);
}


}  // end namespace chrono
