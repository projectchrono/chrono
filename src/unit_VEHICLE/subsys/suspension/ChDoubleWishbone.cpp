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
// with X pointing towards the rear, Y to the right, and Z up. By default, a
// right suspension is constructed.  This can be mirrored to obtain a left
// suspension. Note that this is done by reflecting the y coordinates of the
// hardpoints. As such, the orientation of the suspension reference frame must
// be as specified above. However, its location relative to the chassis is
// arbitrary (and left up to a derived class).
//
// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
// element and its connection to the spindle body (which provides the interface
// to the powertrain subsystem).
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "subsys/suspension/ChDoubleWishbone.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishbone::ChDoubleWishbone(const std::string& name,
                                                 ChSuspension::Side side,
                                                 bool               driven)
: ChSuspension(name, side, driven)
{
  // Create the upright and spindle bodies
  m_spindle = ChSharedBodyPtr(new ChBody);
  m_spindle->SetNameString(name + "_spindle");

  m_upright = ChSharedBodyPtr(new ChBody);
  m_upright->SetNameString(name + "_upright");

  // Revolute joint between spindle and upright
  m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute->SetNameString(name + "_revolute");

  // Bodies and constraints to model upper control arm
  m_UCA = ChSharedBodyPtr(new ChBody);
  m_UCA->SetNameString(name + "_UCA");
  m_revoluteUCA = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteUCA->SetNameString(name + "_revoluteUCA");
  m_sphericalUCA = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUCA->SetNameString(name + "_sphericalUCA");

  // Bodies and constraints to model lower control arm
  m_LCA = ChSharedBodyPtr(new ChBody);
  m_LCA->SetNameString(name + "_LCA");
  m_revoluteLCA = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteLCA->SetNameString(name + "_revoluteLCA");
  m_sphericalLCA = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLCA->SetNameString(name + "_sphericalLCA");

  // Distance constraint to model the tierod
  m_distTierod = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod->SetNameString(name + "_distTierod");

  // Spring-damper
  m_shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_shock->SetNameString(name + "_shock");

  // If driven, create the axle shaft and its connection to the spindle.
  if (m_driven) {
    m_axle = ChSharedPtr<ChShaft>(new ChShaft);
    m_axle->SetNameString(name + "_axle");

    m_axle_to_spindle = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
    m_axle_to_spindle->SetNameString(name + "_axle_to_spindle");
  }
}


// -----------------------------------------------------------------------------
// TODO: does it make sense to ask for a complete Coordsys (i.e. allow for a 
// non-identity rotation) when attaching a suspension subsystem to the chassis?
// -----------------------------------------------------------------------------
void
ChDoubleWishbone::Initialize(ChSharedBodyPtr   chassis,
                                    const ChVector<>& location)
{
  // Transform all points to absolute frame
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    if (m_side == LEFT)
      rel_pos.y = -rel_pos.y;
    m_points[i] = chassis->GetCoord().TransformLocalToParent(location + rel_pos);
  }

  // Set body positions and rotations, mass properties, etc.
  m_spindle->SetPos(m_points[SPINDLE]);
  m_spindle->SetRot(chassis->GetCoord().rot);
  m_spindle->SetMass(getSpindleMass());
  m_spindle->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle();
  OnInitializeSpindle();
  chassis->GetSystem()->AddBody(m_spindle);

  m_UCA->SetPos(m_points[UCA_CM]);
  // Determine the rotation matrix of the UCA based on the plane of the hard points
  ChVector<> wRot;
  wRot.Cross(m_points[UCA_F]-m_points[UCA_U], m_points[UCA_B]-m_points[UCA_U]);
  wRot.Normalize();
  ChVector<> uRot = m_points[UCA_B]-m_points[UCA_F];
  uRot.Normalize();
  ChVector<> vRot;
  vRot.Cross(wRot,uRot);
  ChMatrix33<> rotationMatrix;
  rotationMatrix.Set_A_axis(uRot,vRot,wRot);
  m_UCA->SetRot(rotationMatrix); // Set the rotation of the UCA
  m_UCA->SetMass(getUCAMass());
  m_UCA->SetInertiaXX(getUCAInertia());
  AddVisualizationUCA();
  OnInitializeUCA();
  chassis->GetSystem()->AddBody(m_UCA);

  m_LCA->SetPos(m_points[LCA_CM]);
  // Determine the rotation matrix of the LCA based on the plane of the hard points
  wRot.Cross(m_points[LCA_F]-m_points[LCA_U], m_points[LCA_B]-m_points[LCA_U]);
  wRot.Normalize();
  uRot = m_points[LCA_B]-m_points[LCA_F];
  uRot.Normalize();
  vRot.Cross(wRot,uRot);
  rotationMatrix.Set_A_axis(uRot,vRot,wRot);
  m_LCA->SetRot(rotationMatrix); // Set the rotation of the LCA
  m_LCA->SetMass(getLCAMass());
  m_LCA->SetInertiaXX(getLCAInertia());
  AddVisualizationLCA();
  OnInitializeLCA();
  chassis->GetSystem()->AddBody(m_LCA);

  m_upright->SetPos(m_points[UPRIGHT]);
  m_upright->SetRot(chassis->GetCoord().rot);
  m_upright->SetMass(getUprightMass());
  m_upright->SetInertiaXX(getUprightInertia());
  AddVisualizationUpright();
  OnInitializeUpright();
  chassis->GetSystem()->AddBody(m_upright);

  // Initialize joints
  ChCoordsys<> rev_csys((m_points[UPRIGHT] + m_points[SPINDLE]) / 2, Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute->Initialize(m_spindle, m_upright, rev_csys);
  chassis->GetSystem()->AddLink(m_revolute);

  // Determine the rotation matrix of the UCA based on the plane of the hard points
  wRot = m_points[UCA_B]-m_points[UCA_F];
  wRot.Normalize();
  uRot = ChVector<>(0,0,1);
  uRot.Normalize();
  vRot.Cross(wRot,uRot);
  rotationMatrix.Set_A_axis(uRot,vRot,wRot);
  m_revoluteUCA->Initialize(chassis, m_UCA, ChCoordsys<>((m_points[UCA_F]+m_points[UCA_B])/2, rotationMatrix.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteUCA);
  m_sphericalUCA->Initialize(m_UCA, m_upright, ChCoordsys<>(m_points[UCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUCA);

  // Determine the rotation matrix of the LCA based on the plane of the hard points
  wRot = m_points[LCA_B]-m_points[LCA_F];
  wRot.Normalize();
  uRot = ChVector<>(0,0,1);
  uRot.Normalize();
  vRot.Cross(wRot,uRot);
  rotationMatrix.Set_A_axis(uRot,vRot,wRot);
  m_revoluteLCA->Initialize(chassis, m_LCA, ChCoordsys<>((m_points[LCA_F]+m_points[LCA_B])/2, rotationMatrix.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteLCA);
  m_sphericalLCA->Initialize(m_LCA, m_upright, ChCoordsys<>(m_points[LCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLCA);

  m_distTierod->Initialize(chassis, m_upright, false, m_points[TIEROD_C], m_points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod);

  // Initialize the spring/damper
  m_shock->Initialize(chassis, m_LCA, false, m_points[SHOCK_C], m_points[SHOCK_U]);
  m_shock->Set_SpringK(getSpringCoefficient());
  m_shock->Set_SpringR(getDampingCoefficient());
  m_shock->Set_SpringRestLength(getSpringRestLength());
  chassis->GetSystem()->AddLink(m_shock);

  // Save initial relative position of marker 1 of the tierod distance link,
  // to be used in steering.
  m_tierod_marker = m_distTierod->GetEndPoint1Rel();

  // Initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  if (m_driven) {
    m_axle->SetInertia(getAxleInertia());
    chassis->GetSystem()->Add(m_axle);
    m_axle_to_spindle->Initialize(m_axle, m_spindle, ChVector<>(0, 1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle);
  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::AddVisualizationLCA()
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = m_LCA->TransformPointParentToLocal(m_points[LCA_F]);
  ChVector<> p_B = m_LCA->TransformPointParentToLocal(m_points[LCA_B]);
  ChVector<> p_U = m_LCA->TransformPointParentToLocal(m_points[LCA_U]);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = getLCARadius();
  m_LCA->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = getLCARadius();
  m_LCA->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (m_side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.4f, 0.4f)); break;
  case LEFT:  col->SetColor(ChColor(0.4f, 0.4f, 0.6f)); break;
  }
  m_LCA->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationUCA()
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = m_UCA->TransformPointParentToLocal(m_points[UCA_F]);
  ChVector<> p_B = m_UCA->TransformPointParentToLocal(m_points[UCA_B]);
  ChVector<> p_U = m_UCA->TransformPointParentToLocal(m_points[UCA_U]);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = getUCARadius();
  m_UCA->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = getUCARadius();
  m_UCA->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (m_side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.4f, 0.4f)); break;
  case LEFT:  col->SetColor(ChColor(0.4f, 0.4f, 0.6f)); break;
  }
  m_UCA->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationUpright()
{
  // Express hardpoint locations in body frame.
  ChVector<> p_U = m_upright->TransformPointParentToLocal(m_points[UCA_U]);
  ChVector<> p_L = m_upright->TransformPointParentToLocal(m_points[LCA_U]);
  ChVector<> p_T = m_upright->TransformPointParentToLocal(m_points[TIEROD_U]);

  ChSharedPtr<ChCylinderShape> cyl_L(new ChCylinderShape);
  cyl_L->GetCylinderGeometry().p1 = p_L;
  cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_L->GetCylinderGeometry().rad = getUprightRadius();
  m_upright->AddAsset(cyl_L);
  
  ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
  cyl_U->GetCylinderGeometry().p1 = p_U;
  cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_U->GetCylinderGeometry().rad = getUprightRadius();
  m_upright->AddAsset(cyl_U);

  ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
  cyl_T->GetCylinderGeometry().p1 = p_T;
  cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_T->GetCylinderGeometry().rad = getUprightRadius();
  m_upright->AddAsset(cyl_T);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (m_side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.1f, 0.1f)); break;
  case LEFT:  col->SetColor(ChColor(0.1f, 0.1f, 0.6f)); break;
  }
  m_upright->AddAsset(col);
}

void ChDoubleWishbone::AddVisualizationSpindle()
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().rad = getSpindleRadius();
  m_spindle->AddAsset(cyl);

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChDoubleWishbone::ApplySteering(double displ)
{
  ChVector<> r_bar = m_tierod_marker;
  r_bar.y += displ;
  m_distTierod->SetEndPoint1Rel(r_bar);
}


} // end namespace chrono
