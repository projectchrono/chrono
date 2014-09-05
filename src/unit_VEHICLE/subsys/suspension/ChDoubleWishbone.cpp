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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Base class for a double-A arm suspension modeled with bodies and constraints.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the rear, Y to the right, and Z up. All point
// locations are assumed to be given for the right half of the supspension and
// will be mirrored (reflecting the y coordinates) to construct the left side.
//
// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
// element and its connection to the spindle body (which provides the interface
// to the driveline subsystem).
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "subsys/suspension/ChDoubleWishbone.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChDoubleWishbone::m_pointNames[] = {
  "SPINDLE ",
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
  "TIEROD_U"
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishbone::ChDoubleWishbone(const std::string& name,
                                   bool               steerable,
                                   bool               driven)
: ChSuspension(name, steerable, driven)
{
  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

void ChDoubleWishbone::CreateSide(ChSuspension::Side side,
                                  const std::string& suffix)
{
  // Create the spindle and upright bodies
  m_spindle[side] = ChSharedBodyPtr(new ChBody);
  m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
  m_upright[side] = ChSharedBodyPtr(new ChBody);
  m_upright[side]->SetNameString(m_name + "_upright" + suffix);

  // Revolute joint between spindle and upright
  m_revolute[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);

  // Bodies and constraints to model upper control arm
  m_UCA[side] = ChSharedBodyPtr(new ChBody);
  m_UCA[side]->SetNameString(m_name + "_UCA" + suffix);
  m_revoluteUCA[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteUCA[side]->SetNameString(m_name + "_revoluteUCA" + suffix);
  m_sphericalUCA[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUCA[side]->SetNameString(m_name + "_sphericalUCA" + suffix);

  // Bodies and constraints to model lower control arm
  m_LCA[side] = ChSharedBodyPtr(new ChBody);
  m_LCA[side]->SetNameString(m_name + "_LCA" + suffix);
  m_revoluteLCA[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteLCA[side]->SetNameString(m_name + "_revoluteLCA" + suffix);
  m_sphericalLCA[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLCA[side]->SetNameString(m_name + "_sphericalLCA" + suffix);

  // Distance constraint to model the tierod
  m_distTierod[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);

  // Spring-damper
  m_shock[side] = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_shock[side]->SetNameString(m_name + "_shock" + suffix);
  m_spring[side] = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_spring[side]->SetNameString(m_name + "_spring" + suffix);

  // If driven, create the axle shaft and its connection to the spindle.
  if (m_driven) {
    m_axle[side] = ChSharedPtr<ChShaft>(new ChShaft);
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle_to_spindle[side] = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::Initialize(ChSharedBodyPtr   chassis,
                                  const ChVector<>& location)
{
  // Transform all points to absolute frame and initialize left side.
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    rel_pos.y = -rel_pos.y;
    m_points[i] = chassis->GetCoord().TransformLocalToParent(location + rel_pos);
  }

  InitializeSide(LEFT, chassis);

  // Transform all points to absolute frame and initialize right side.
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    m_points[i] = chassis->GetCoord().TransformLocalToParent(location + rel_pos);
  }

  InitializeSide(RIGHT, chassis);
}

void ChDoubleWishbone::InitializeSide(ChSuspension::Side side,
                                      ChSharedBodyPtr    chassis)
{
  // Initialize spindle body.
  m_spindle[side]->SetPos(m_points[SPINDLE]);
  m_spindle[side]->SetRot(chassis->GetCoord().rot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(side);
  OnInitializeSpindle(side);
  chassis->GetSystem()->AddBody(m_spindle[side]);

  // Initialize upright body.
  m_upright[side]->SetPos(m_points[UPRIGHT]);
  m_upright[side]->SetRot(chassis->GetCoord().rot);
  m_upright[side]->SetMass(getUprightMass());
  m_upright[side]->SetInertiaXX(getUprightInertia());
  AddVisualizationUpright(side);
  OnInitializeUpright(side);
  chassis->GetSystem()->AddBody(m_upright[side]);

  // Unit vectors for orientation matrices.
  ChVector<> u;
  ChVector<> v;
  ChVector<> w;
  ChMatrix33<> rot;

  // Initialize Upper Control Arm body.
  // Determine the rotation matrix of the UCA based on the plane of the hard points
  // (z axis normal to the plane of the LCA)
  w = Vcross(m_points[UCA_F] - m_points[UCA_U], m_points[UCA_B] - m_points[UCA_U]);
  w.Normalize();
  u = m_points[UCA_B] - m_points[UCA_F];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_UCA[side]->SetPos(m_points[UCA_CM]);
  m_UCA[side]->SetRot(rot);
  m_UCA[side]->SetMass(getUCAMass());
  m_UCA[side]->SetInertiaXX(getUCAInertia());
  AddVisualizationUCA(side);
  OnInitializeUCA(side);
  chassis->GetSystem()->AddBody(m_UCA[side]);

  // Initialize Lower Control Arm body.
  // Determine the rotation matrix of the LCA, based on the plane of the hard points
  // (z axis normal to the plane of the LCA)
  w = Vcross(m_points[LCA_F] - m_points[LCA_U], m_points[LCA_B] - m_points[LCA_U]);
  w.Normalize();
  u = m_points[LCA_B] - m_points[LCA_F];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_LCA[side]->SetPos(m_points[LCA_CM]);
  m_LCA[side]->SetRot(rot);
  m_LCA[side]->SetMass(getLCAMass());
  m_LCA[side]->SetInertiaXX(getLCAInertia());
  AddVisualizationLCA(side);
  OnInitializeLCA(side);
  chassis->GetSystem()->AddBody(m_LCA[side]);


  // Initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys((m_points[UPRIGHT] + m_points[SPINDLE]) / 2, Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  // Initialize the revolute joint between chassis and UCA.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the UCA.
  v = Vcross(m_points[UCA_F] - m_points[UCA_U], m_points[UCA_B] - m_points[UCA_U]);
  v.Normalize();
  w = m_points[UCA_B] - m_points[UCA_F];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteUCA[side]->Initialize(chassis, m_UCA[side], ChCoordsys<>((m_points[UCA_F]+m_points[UCA_B])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteUCA[side]);

  // Initialize the spherical joint between upright and UCA.
  m_sphericalUCA[side]->Initialize(m_UCA[side], m_upright[side], ChCoordsys<>(m_points[UCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUCA[side]);

  // Initialize the revolute joint between chassis and LCA.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the LCA.
  v = Vcross(m_points[LCA_F] - m_points[LCA_U], m_points[LCA_B] - m_points[LCA_U]);
  v.Normalize();
  w = m_points[LCA_B] - m_points[LCA_F];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteLCA[side]->Initialize(chassis, m_LCA[side], ChCoordsys<>((m_points[LCA_F]+m_points[LCA_B])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteLCA[side]);

  // Initialize the spherical joint between upright and LCA.
  m_sphericalLCA[side]->Initialize(m_LCA[side], m_upright[side], ChCoordsys<>(m_points[LCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLCA[side]);

  // Initialize the tierod distance constraint between chassis and upright.
  m_distTierod[side]->Initialize(chassis, m_upright[side], false, m_points[TIEROD_C], m_points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_LCA[side], false, m_points[SHOCK_C], m_points[SHOCK_A], false, getSpringRestLength());
  m_shock[side]->Set_SpringK(0.0);
  m_shock[side]->Set_SpringR(getDampingCoefficient());
  chassis->GetSystem()->AddLink(m_shock[side]);

  m_spring[side]->Initialize(chassis, m_LCA[side], false, m_points[SPRING_C], m_points[SPRING_A], false, getSpringRestLength());
  m_spring[side]->Set_SpringK(getSpringCoefficient());
  m_spring[side]->Set_SpringR(0.0);
  chassis->GetSystem()->AddLink(m_spring[side]);

  // Save initial relative position of marker 1 of the tierod distance link,
  // to be used in steering.
  m_tierod_marker[side] = m_distTierod[side]->GetEndPoint1Rel();

  // Initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  if (m_driven) {
    m_axle[side]->SetInertia(getAxleInertia());
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, 1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChDoubleWishbone::GetSpringForce(ChSuspension::Side side)
{
  return  m_spring[side]->Get_SpringReact();
}

double ChDoubleWishbone::GetSpringLen(ChSuspension::Side side)
{
  return (m_spring[side]->GetMarker1()->GetAbsCoord().pos - m_spring[side]->GetMarker2()->GetAbsCoord().pos).Length();
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::LogHardpointLocations(const ChVector<>& ref,
                                             bool              inches)
{
  double unit = inches ? 1 / 0.0254 : 1.0;

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

    GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x << "  " << pos.y << "  " << pos.z << "\n";
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::LogConstraintViolations(ChSuspension::Side side)
{
  // Revolute joints
  {
    ChMatrix<>* C = m_revoluteLCA[side]->GetC();
    GetLog() << "LCA revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "\n";
    GetLog() << "  " << C->GetElement(1, 0) << "\n";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_revoluteUCA[side]->GetC();
    GetLog() << "UCA revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "\n";
    GetLog() << "  " << C->GetElement(1, 0) << "\n";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_revolute[side]->GetC();
    GetLog() << "Spindle revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "\n";
    GetLog() << "  " << C->GetElement(1, 0) << "\n";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Spherical joints
  {
    ChMatrix<>* C = m_sphericalLCA[side]->GetC();
    GetLog() << "LCA spherical\n";
    GetLog() << "  " << C->GetElement(0, 0) << "\n";
    GetLog() << "  " << C->GetElement(1, 0) << "\n";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_sphericalUCA[side]->GetC();
    GetLog() << "UCA spherical\n";
    GetLog() << "  " << C->GetElement(0, 0) << "\n";
    GetLog() << "  " << C->GetElement(1, 0) << "\n";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }

  // Distance constraint
  GetLog() << "Tierod distance\n";
  GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::AddVisualizationLCA(ChSuspension::Side side)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = m_LCA[side]->TransformPointParentToLocal(m_points[LCA_F]);
  ChVector<> p_B = m_LCA[side]->TransformPointParentToLocal(m_points[LCA_B]);
  ChVector<> p_U = m_LCA[side]->TransformPointParentToLocal(m_points[LCA_U]);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = getLCARadius();
  m_LCA[side]->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = getLCARadius();
  m_LCA[side]->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.4f, 0.4f)); break;
  case LEFT:  col->SetColor(ChColor(0.4f, 0.4f, 0.6f)); break;
  }
  m_LCA[side]->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationUCA(ChSuspension::Side side)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = m_UCA[side]->TransformPointParentToLocal(m_points[UCA_F]);
  ChVector<> p_B = m_UCA[side]->TransformPointParentToLocal(m_points[UCA_B]);
  ChVector<> p_U = m_UCA[side]->TransformPointParentToLocal(m_points[UCA_U]);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = getUCARadius();
  m_UCA[side]->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = getUCARadius();
  m_UCA[side]->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.4f, 0.4f)); break;
  case LEFT:  col->SetColor(ChColor(0.4f, 0.4f, 0.6f)); break;
  }
  m_UCA[side]->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationUpright(ChSuspension::Side side)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_U = m_upright[side]->TransformPointParentToLocal(m_points[UCA_U]);
  ChVector<> p_L = m_upright[side]->TransformPointParentToLocal(m_points[LCA_U]);
  ChVector<> p_T = m_upright[side]->TransformPointParentToLocal(m_points[TIEROD_U]);

  ChSharedPtr<ChCylinderShape> cyl_L(new ChCylinderShape);
  cyl_L->GetCylinderGeometry().p1 = p_L;
  cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_L->GetCylinderGeometry().rad = getUprightRadius();
  m_upright[side]->AddAsset(cyl_L);
  
  ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
  cyl_U->GetCylinderGeometry().p1 = p_U;
  cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_U->GetCylinderGeometry().rad = getUprightRadius();
  m_upright[side]->AddAsset(cyl_U);

  ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
  cyl_T->GetCylinderGeometry().p1 = p_T;
  cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_T->GetCylinderGeometry().rad = getUprightRadius();
  m_upright[side]->AddAsset(cyl_T);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.1f, 0.1f)); break;
  case LEFT:  col->SetColor(ChColor(0.1f, 0.1f, 0.6f)); break;
  }
  m_upright[side]->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationSpindle(ChSuspension::Side side)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().rad = getSpindleRadius();
  m_spindle[side]->AddAsset(cyl);

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::ApplySteering(double displ)
{
  {
    ChVector<> r_bar = m_tierod_marker[LEFT];
    r_bar.y += displ;
    m_distTierod[LEFT]->SetEndPoint1Rel(r_bar);
  }
  {
    ChVector<> r_bar = m_tierod_marker[RIGHT];
    r_bar.y += displ;
    m_distTierod[RIGHT]->SetEndPoint1Rel(r_bar);
  }
}


} // end namespace chrono