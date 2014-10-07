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
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
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
                                                 bool               driven)
: ChSuspension(name, driven)
{
  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

void ChDoubleWishboneReduced::CreateSide(ChVehicleSide      side,
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
void ChDoubleWishboneReduced::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                                         const ChVector<>&          location,
                                         ChSharedPtr<ChBody>        tierod_body)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Transform all points to absolute frame and initialize left side.
  std::vector<ChVector<> > points(NUM_POINTS);

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    points[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  InitializeSide(LEFT, chassis, tierod_body, points);

  // Transform all points to absolute frame and initialize right side.
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    rel_pos.y = -rel_pos.y;
    points[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  InitializeSide(RIGHT, chassis, tierod_body, points);
}


void ChDoubleWishboneReduced::InitializeSide(ChVehicleSide                   side,
                                             ChSharedPtr<ChBodyAuxRef>       chassis,
                                             ChSharedPtr<ChBody>             tierod_body,
                                             const std::vector<ChVector<> >& points)
{
  // Chassis orientation (expressed in absolute frame)
  // Recall that the suspension reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Initialize spindle body (same orientation as the chassis)
  m_spindle[side]->SetPos(points[SPINDLE]);
  m_spindle[side]->SetRot(chassisRot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(m_spindle[side], getSpindleRadius(), getSpindleWidth());
  chassis->GetSystem()->AddBody(m_spindle[side]);

  // Initialize upright body (same orientation as the chassis)
  m_upright[side]->SetPos(points[UPRIGHT]);
  m_upright[side]->SetRot(chassisRot);
  m_upright[side]->SetMass(getUprightMass());
  m_upright[side]->SetInertiaXX(getUprightInertia());
  AddVisualizationUpright(m_upright[side], points[UCA_U], points[LCA_U], points[TIEROD_U], getUprightRadius());
  chassis->GetSystem()->AddBody(m_upright[side]);

  // Initialize joints
  ChCoordsys<> rev_csys((points[UPRIGHT] + points[SPINDLE]) / 2, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  m_distUCA_F[side]->Initialize(chassis, m_upright[side], false, points[UCA_F], points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_F[side]);
  m_distUCA_B[side]->Initialize(chassis, m_upright[side], false, points[UCA_B], points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_B[side]);

  m_distLCA_F[side]->Initialize(chassis, m_upright[side], false, points[LCA_F], points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_F[side]);
  m_distLCA_B[side]->Initialize(chassis, m_upright[side], false, points[LCA_B], points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_B[side]);

  m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
  m_shock[side]->Set_SpringK(getSpringCoefficient());
  m_shock[side]->Set_SpringR(getDampingCoefficient());
  m_shock[side]->Set_SpringRestLength(getSpringRestLength());
  chassis->GetSystem()->AddLink(m_shock[side]);

  // Initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  if (m_driven) {
    m_axle[side]->SetInertia(getAxleInertia());
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::AddVisualizationUpright(ChSharedBodyPtr    upright,
                                                      const ChVector<>&  pt_U,
                                                      const ChVector<>&  pt_L,
                                                      const ChVector<>&  pt_T,
                                                      double             radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);
  ChVector<> p_L = upright->TransformPointParentToLocal(pt_L);
  ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);

  ChSharedPtr<ChCylinderShape> cyl_L(new ChCylinderShape);
  cyl_L->GetCylinderGeometry().p1 = p_L;
  cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_L->GetCylinderGeometry().rad = radius;
  upright->AddAsset(cyl_L);

  ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
  cyl_U->GetCylinderGeometry().p1 = p_U;
  cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_U->GetCylinderGeometry().rad = radius;
  upright->AddAsset(cyl_U);

  ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
  cyl_T->GetCylinderGeometry().p1 = p_T;
  cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_T->GetCylinderGeometry().rad = radius;
  upright->AddAsset(cyl_T);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  upright->AddAsset(col);
}

void ChDoubleWishboneReduced::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                                      double          radius,
                                                      double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::LogConstraintViolations(ChVehicleSide side)
{
  // Revolute joint
  {
    ChMatrix<>* C = m_revolute[side]->GetC();
    GetLog() << "Spindle revolute      ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Distance constraints
  GetLog() << "UCA front distance    ";
  GetLog() << "  " << m_distUCA_F[side]->GetCurrentDistance() - m_distUCA_F[side]->GetImposedDistance() << "\n";

  GetLog() << "UCA back distance     ";
  GetLog() << "  " << m_distUCA_B[side]->GetCurrentDistance() - m_distUCA_B[side]->GetImposedDistance() << "\n";

  GetLog() << "LCA front distance    ";
  GetLog() << "  " << m_distLCA_F[side]->GetCurrentDistance() - m_distLCA_F[side]->GetImposedDistance() << "\n";

  GetLog() << "LCA back distance     ";
  GetLog() << "  " << m_distLCA_B[side]->GetCurrentDistance() - m_distLCA_B[side]->GetImposedDistance() << "\n";

  GetLog() << "Tierod distance       ";
  GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";

}


} // end namespace chrono
