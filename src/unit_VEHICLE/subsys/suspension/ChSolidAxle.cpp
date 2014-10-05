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
// Base class for a solid axle suspension modeled with bodies and constraints.
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

#include "subsys/suspension/ChSolidAxle.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSolidAxle::m_pointNames[] = {
    "SHOCK_A    ",
    "SHOCK_C    ",
    "KNUCKLE_L  ",
    "KNUCKLE_U  ",
    "LL_A       ",
    "LL_C       ",
    "UL_A       ",
    "UL_C       ",
    "SPRING_A   ",
    "SPRING_C   ",
    "TIEROD_C   ",
    "TIEROD_K   ",
    "SPINDLE    ",
    "KNUCKLE_CM ",
    "LL_CM      ",
    "UL_CM      "
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSolidAxle::ChSolidAxle(const std::string& name,
                         bool               driven)
: ChSuspension(name, driven)
{
  // Create the axle body
  m_axleTube = ChSharedBodyPtr(new ChBody);
  m_axleTube->SetNameString(m_name + "_axleTube");

  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

void ChSolidAxle::CreateSide(ChSuspension::Side side,
                             const std::string& suffix)
{
  // Create the knuckle bodies
  m_knuckle[side] = ChSharedBodyPtr(new ChBody);
  m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);

  // Create the knuckle - axle revolute joints
  m_revoluteKingpin[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);

  // Create the spindle bodies
  m_spindle[side] = ChSharedBodyPtr(new ChBody);
  m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);

  // Create the spindle - knuckle revolute joins
  m_revolute[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);

  // Create the upper link bodies
  m_upperLink[side] = ChSharedBodyPtr(new ChBody);
  m_upperLink[side]->SetNameString(m_name + "_upperLink" + suffix);

  // Create the upper link - axle spherical joints
  m_sphericalUpperLink[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUpperLink[side]->SetNameString(m_name + "_sphericalUpperLink" + suffix);

  // Create the upper link - chassis universal joints
  m_universalUpperLink[side] = ChSharedPtr<ChLinkLockUniversal>(new ChLinkLockUniversal);
  m_universalUpperLink[side]->SetNameString(m_name + "_universalUpperLink" + suffix);

  // Create the lower link bodies
  m_lowerLink[side] = ChSharedBodyPtr(new ChBody);
  m_lowerLink[side]->SetNameString(m_name + "_lowerLink" + suffix);

  // Create the lower link - axle spherical joints
  m_sphericalLowerLink[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLowerLink[side]->SetNameString(m_name + "_sphericalLowerLink" + suffix);

  // Create the lower link - chassis universal joints
  m_universalLowerLink[side] = ChSharedPtr<ChLinkLockUniversal>(new ChLinkLockUniversal);
  m_universalLowerLink[side]->SetNameString(m_name + "_universalLowerLink" + suffix);

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
void ChSolidAxle::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                             const ChVector<>&          location,
                             ChSharedPtr<ChBody>        tierod_body)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Transform the location of the axle body COM to absolute frame.
  ChVector<> axleCOM_local = getAxleTubeCOM();
  ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

  // Calculate end points on the axle body, expressed in the absolute frame
  // (for visualization)
  ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
  ChVector<> outer_local(axleCOM_local.x, midpoint_local.y, axleCOM_local.z);
  ChVector<> axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
  outer_local.y = -outer_local.y;
  ChVector<> axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

  // Initialize axle body.
  m_axleTube->SetPos(axleCOM);
  m_axleTube->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
  m_axleTube->SetMass(getAxleTubeMass());
  m_axleTube->SetInertiaXX(getAxleTubeInertia());
  AddVisualizationLink(m_axleTube, axleOuterL, axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
  chassis->GetSystem()->AddBody(m_axleTube);

  // Transform all points and directions on right and left sides to absolute frame
  std::vector<ChVector<> > points_R(NUM_POINTS);
  std::vector<ChVector<> > points_L(NUM_POINTS);
  std::vector<ChVector<> > dirs_R(NUM_DIRS);
  std::vector<ChVector<> > dirs_L(NUM_DIRS);

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    points_L[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    rel_pos.y = -rel_pos.y;
    points_R[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  for (int i = 0; i < NUM_DIRS; i++) {
    ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
    dirs_L[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
    rel_dir.y = -rel_dir.y;
    dirs_R[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
  }

  // Initialize left and right sides.
  InitializeSide(LEFT, chassis, tierod_body, points_L, dirs_L);
  InitializeSide(RIGHT, chassis, tierod_body, points_R, dirs_R);
}

void ChSolidAxle::InitializeSide(ChSuspension::Side              side,
                                 ChSharedPtr<ChBodyAuxRef>       chassis,
                                 ChSharedPtr<ChBody>             tierod_body,
                                 const std::vector<ChVector<> >& points,
                                 const std::vector<ChVector<> >& dirs)
{
  // Unit vectors for orientation matrices.
  ChVector<> u;
  ChVector<> v;
  ChVector<> w;
  ChMatrix33<> rot;

  // Chassis orientation (expressed in absolute frame)
  // Recall that the suspension reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Initialize knuckle body (same orientation as the chassis)
  m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
  m_knuckle[side]->SetRot(chassisRot);
  m_knuckle[side]->SetMass(getKnuckleMass());
  m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
  AddVisualizationKnuckle(m_knuckle[side], points[KNUCKLE_U], points[KNUCKLE_L], points[TIEROD_K], getKnuckleRadius());
  chassis->GetSystem()->AddBody(m_knuckle[side]);

  // Initialize spindle body (same orientation as the chassis)
  m_spindle[side]->SetPos(points[SPINDLE]);
  m_spindle[side]->SetRot(chassisRot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(m_spindle[side], getSpindleRadius(), getSpindleWidth());
  chassis->GetSystem()->AddBody(m_spindle[side]);

  // Initialize upper link body.
  // Determine the rotation matrix of the upper link based on the plane of the hard points
  // (z-axis along the length of the upper link)
  v = Vcross(points[UL_A] - points[LL_A], points[UL_C] - points[LL_A]);
  v.Normalize();
  w = points[UL_C] - points[UL_A];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_upperLink[side]->SetPos(points[UL_CM]);
  m_upperLink[side]->SetRot(rot);
  m_upperLink[side]->SetMass(getULMass());
  m_upperLink[side]->SetInertiaXX(getULInertia());
  AddVisualizationLink(m_upperLink[side], points[UL_A], points[UL_C], getULRadius(), ChColor(0.6f, 0.2f, 0.6f));
  chassis->GetSystem()->AddBody(m_upperLink[side]);

  // Initialize lower link body.
  // Determine the rotation matrix of the lower link based on the plane of the hard points
  // (z-axis along the length of the lower link)
  v = Vcross(points[LL_C] - points[UL_A], points[LL_A] - points[UL_A]);
  v.Normalize();
  w = points[LL_C] - points[LL_A];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_lowerLink[side]->SetPos(points[LL_CM]);
  m_lowerLink[side]->SetRot(rot);
  m_lowerLink[side]->SetMass(getLLMass());
  m_lowerLink[side]->SetInertiaXX(getLLInertia());
  AddVisualizationLink(m_lowerLink[side], points[LL_A], points[LL_C], getLLRadius(), ChColor(0.2f, 0.6f, 0.2f));
  chassis->GetSystem()->AddBody(m_lowerLink[side]);

  // Append to the axle visualization
  AddVisualizationLink(m_axleTube, points[LL_A], points[UL_A], getLLRadius(), ChColor(0.7f, 0.7f, 0.7f));

  // Initialize the revolute joint between axle and knuckle.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction.
  w = points[KNUCKLE_U] - points[KNUCKLE_L];
  w.Normalize();
  u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_revoluteKingpin[side]->Initialize(m_axleTube, m_knuckle[side], ChCoordsys<>((points[KNUCKLE_U]+points[KNUCKLE_L])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

  // Initialize the spherical joint between axle and upper link.
  m_sphericalUpperLink[side]->Initialize(m_axleTube, m_upperLink[side], ChCoordsys<>(points[UL_A], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUpperLink[side]);

  // Initialize the spherical joint between axle and lower link.
  m_sphericalLowerLink[side]->Initialize(m_axleTube, m_lowerLink[side], ChCoordsys<>(points[LL_A], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLowerLink[side]);

  // Initialize the universal joint between chassis and upper link.
  u = dirs[UNIV_AXIS_CHASSIS_U];
  v = dirs[UNIV_AXIS_LINK_U];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);
  m_universalUpperLink[side]->Initialize(chassis, m_upperLink[side], ChCoordsys<>(points[UL_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalUpperLink[side]);

  // Initialize the universal joint between chassis and lower link.
  u = dirs[UNIV_AXIS_CHASSIS_L];
  v = dirs[UNIV_AXIS_LINK_L];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);
  m_universalLowerLink[side]->Initialize(chassis, m_lowerLink[side], ChCoordsys<>(points[LL_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalLowerLink[side]);

  // Initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A], false, getSpringRestLength());
  m_shock[side]->Set_SpringK(0.0);
  m_shock[side]->Set_SpringR(getDampingCoefficient());
  chassis->GetSystem()->AddLink(m_shock[side]);

  m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A], false, getSpringRestLength());
  m_spring[side]->Set_SpringK(getSpringCoefficient());
  m_spring[side]->Set_SpringR(0.0);
  chassis->GetSystem()->AddLink(m_spring[side]);

  // Initialize the tierod distance constraint between chassis and upright.
  m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_K]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

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
void ChSolidAxle::LogHardpointLocations(const ChVector<>& ref,
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
void ChSolidAxle::LogConstraintViolations(ChSuspension::Side side)
{
  /*
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
  */
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::AddVisualizationLink(ChSharedBodyPtr    body,
                                       const ChVector<>&  pt_1,
                                       const ChVector<>&  pt_2,
                                       double             radius,
                                       const ChColor&     color)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
  ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = p_1;
  cyl->GetCylinderGeometry().p2 = p_2;
  cyl->GetCylinderGeometry().rad = radius;
  body->AddAsset(cyl);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(color);
  body->AddAsset(col);
}

void ChSolidAxle::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                          double          radius,
                                          double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}

void ChSolidAxle::AddVisualizationKnuckle(ChSharedBodyPtr knuckle,
                                          const ChVector<>&  pt_U,
                                          const ChVector<>&  pt_L,
                                          const ChVector<>&  pt_T,
                                          double             radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
  ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
  ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

  ChSharedPtr<ChCylinderShape> cyl_L(new ChCylinderShape);
  cyl_L->GetCylinderGeometry().p1 = p_L;
  cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_L->GetCylinderGeometry().rad = radius;
  knuckle->AddAsset(cyl_L);

  ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
  cyl_U->GetCylinderGeometry().p1 = p_U;
  cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_U->GetCylinderGeometry().rad = radius;
  knuckle->AddAsset(cyl_U);

  ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
  cyl_T->GetCylinderGeometry().p1 = p_T;
  cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
  cyl_T->GetCylinderGeometry().rad = radius;
  knuckle->AddAsset(cyl_T);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  knuckle->AddAsset(col);
}


} // end namespace chrono
