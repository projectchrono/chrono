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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a double-A arm suspension modeled with distance constraints.
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

#include "subsys/suspension/ChDoubleWishboneReduced.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishboneReduced::ChDoubleWishboneReduced(const std::string& name,
                                                 bool               steerable,
                                                 bool               driven)
: ChSuspension(name, steerable, driven)
{
  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

void ChDoubleWishboneReduced::CreateSide(ChSuspension::Side side,
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

  // Distance constraints to model upper control arm
  m_distUCA_F[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distUCA_F[side]->SetNameString(m_name + "_distUCA_F" + suffix);
  m_distUCA_B[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distUCA_B[side]->SetNameString(m_name + "_distUCA_B" + suffix);

  // Distance constraints to model lower control arm
  m_distLCA_F[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distLCA_F[side]->SetNameString(m_name + "_distLCA_F" + suffix);
  m_distLCA_B[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distLCA_B[side]->SetNameString(m_name + "_distLCA_B" + suffix);

  // Distance constraint to model the tierod
  m_distTierod[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);

  // Spring-damper
  m_shock[side] = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_shock[side]->SetNameString(m_name + "_shock" + suffix);

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
void ChDoubleWishboneReduced::Initialize(ChSharedBodyPtr   chassis,
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


void ChDoubleWishboneReduced::InitializeSide(ChSuspension::Side side,
                                             ChSharedBodyPtr    chassis)
{
  // Set body positions and rotations, mass properties, etc.
  m_spindle[side]->SetPos(m_points[SPINDLE]);
  m_spindle[side]->SetRot(chassis->GetCoord().rot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(side);
  OnInitializeSpindle(side);
  chassis->GetSystem()->AddBody(m_spindle[side]);

  m_upright[side]->SetPos(m_points[UPRIGHT]);
  m_upright[side]->SetRot(chassis->GetCoord().rot);
  m_upright[side]->SetMass(getUprightMass());
  m_upright[side]->SetInertiaXX(getUprightInertia());
  AddVisualizationUpright(side);
  OnInitializeUpright(side);
  chassis->GetSystem()->AddBody(m_upright[side]);

  // Initialize joints
  ChCoordsys<> rev_csys((m_points[UPRIGHT] + m_points[SPINDLE]) / 2, Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  m_distUCA_F[side]->Initialize(chassis, m_upright[side], false, m_points[UCA_F], m_points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_F[side]);
  m_distUCA_B[side]->Initialize(chassis, m_upright[side], false, m_points[UCA_B], m_points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_B[side]);

  m_distLCA_F[side]->Initialize(chassis, m_upright[side], false, m_points[LCA_F], m_points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_F[side]);
  m_distLCA_B[side]->Initialize(chassis, m_upright[side], false, m_points[LCA_B], m_points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_B[side]);

  m_distTierod[side]->Initialize(chassis, m_upright[side], false, m_points[TIEROD_C], m_points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_upright[side], false, m_points[SHOCK_C], m_points[SHOCK_U]);
  m_shock[side]->Set_SpringK(getSpringCoefficient());
  m_shock[side]->Set_SpringR(getDampingCoefficient());
  m_shock[side]->Set_SpringRestLength(getSpringRestLength());
  chassis->GetSystem()->AddLink(m_shock[side]);

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
void ChDoubleWishboneReduced::AddVisualizationUpright(ChSuspension::Side side)
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

void ChDoubleWishboneReduced::AddVisualizationSpindle(ChSuspension::Side side)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -getSpindleWidth() / 2, 0);
  cyl->GetCylinderGeometry().rad = getSpindleRadius();
  m_spindle[side]->AddAsset(cyl);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::ApplySteering(double displ)
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
