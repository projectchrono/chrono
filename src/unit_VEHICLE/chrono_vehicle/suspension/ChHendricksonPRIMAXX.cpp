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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
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

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "chrono_vehicle/suspension/ChHendricksonPRIMAXX.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChHendricksonPRIMAXX::m_pointNames[] = {
  "SPINDLE ",
  "KNUCKLE_L",
  "KNUCKLE_U",
  "TIEROD_C",
  "TIEROD_K",
  "TORQUEROD_C",
  "TORQUEROD_AH",
  "LOWERBEAM_C",
  "LOWERBEAM_AH",
  "LOWERBEAM_TB",
  "SHOCKAH_C",
  "SHOCKAH_AH",
  "SPRINGAH_C",
  "SPRINGAH_AH",
  "SHOCKLB_C",
  "SHOCKLB_LB",
  "SPRINGLB_C",
  "SPRINGLB_LB",
  "KNUCKLE_CM",
  "TORQUEROD_CM",
  "LOWERBEAM_CM",
  "SPRINGAH_CM",
  "SPRINGLB_CM",
  "TRANSVERSEBEAM_CM",
  "AXLEHOUSING_CM",
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChHendricksonPRIMAXX::ChHendricksonPRIMAXX(const std::string& name)
: ChSuspension(name)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                                  const ChVector<>&          location,
                                  ChSharedPtr<ChBody>        tierod_body)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  ///* TODO  AXLE STUFF
  ///*
  ///* AxleTube is in this template axlehousing
  ///*
  // Transform the location of the axle body COM to absolute frame.
  ChVector<> axleCOM_local = getAxlehousingCOM();
  ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

  // Calculate end points on the axle body, expressed in the absolute frame
  // (for visualization)
  ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
  ChVector<> outer_local(axleCOM_local.x, midpoint_local.y, axleCOM_local.z);
  ChVector<> axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
  outer_local.y = -outer_local.y;
  ChVector<> axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

  // Create and initialize the axlehousing body.
  m_axlehousing = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_axlehousing->SetNameString(m_name + "_axlehousing");
  m_axlehousing->SetPos(axleCOM);
  m_axlehousing->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
  m_axlehousing->SetMass(getAxlehousingMass());
  m_axlehousing->SetInertiaXX(getAxlehousingInertia());
  AddVisualizationLink(m_axlehousing, axleOuterL, axleOuterR, getAxlehousingRadius(), ChColor(0.7f, 0.7f, 0.7f));
  chassis->GetSystem()->AddBody(m_axlehousing);

  ///*
  ///*ENDOF AXLE STUFF

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

void ChHendricksonPRIMAXX::InitializeSide(ChVehicleSide                   side,
                                          ChSharedPtr<ChBodyAuxRef>       chassis,
                                          ChSharedPtr<ChBody>             tierod_body,
                                          const std::vector<ChVector<> >& points,
                                          const std::vector<ChVector<> >& dirs)
{
  std::string suffix = (side == LEFT) ? "_L" : "_R";


// Unit vectors for orientation matrices.
ChVector<> u;
ChVector<> v;
ChVector<> w;
ChMatrix33<> rot;

  
  // Chassis orientation (expressed in absolute frame)
  // Recall that the suspension reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Create and initialize knuckle body (same orientation as the chassis)
  m_knuckle[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
  m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
  m_knuckle[side]->SetRot(chassisRot);
  m_knuckle[side]->SetMass(getKnuckleMass());
  m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
  AddVisualizationKnuckle(m_knuckle[side], points[KNUCKLE_U], points[KNUCKLE_L], points[TIEROD_K], getKnuckleRadius());
  chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Create and initialize spindle body (same orientation as the chassis)
  m_spindle[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
  m_spindle[side]->SetPos(points[SPINDLE]);
  m_spindle[side]->SetRot(chassisRot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(m_spindle[side], getSpindleRadius(), getSpindleWidth());
  chassis->GetSystem()->AddBody(m_spindle[side]);

  ///*
  ///* TODO create and initialize the new bodies of template
  ///*
  ///*POINTS OF TORQUEROD AND LOWER BEAM TO AXLEHOUSING ARE PROBABLY WRONG
  ///*PLANE DEFINITION WRONG? CHECK WITH CHSOLIDAXLE.cpp


  // Create and initialize torque rod body.
  // Determine the rotation matrix of the torque rod based on the plane of the hard points
  // (z-axis along the length of the torque rod)
  v = Vcross(points[TORQUEROD_AH] - points[LOWERBEAM_AH], points[TORQUEROD_C] - points[LOWERBEAM_AH]);
  v.Normalize();
  w = points[TORQUEROD_C] - points[TORQUEROD_AH];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_torquerod[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_torquerod[side]->SetNameString(m_name + "_torquerod" + suffix);
  m_torquerod[side]->SetPos(points[TORQUEROD_CM]);
  m_torquerod[side]->SetRot(rot);
  m_torquerod[side]->SetMass(getTorquerodMass());
  m_torquerod[side]->SetInertiaXX(getTorquerodInertia());
  AddVisualizationLink(m_torquerod[side], points[TORQUEROD_AH], points[TORQUEROD_C], getTorquerodRadius(), ChColor(0.6f, 0.2f, 0.6f));
  chassis->GetSystem()->AddBody(m_torquerod[side]);

  // Create and initialize lower beam body.
  // Determine the rotation matrix of the lower link based on the plane of the hard points
  // (z-axis along the length of the lower link)
  v = Vcross(points[LOWERBEAM_C] - points[TORQUEROD_AH], points[LOWERBEAM_AH] - points[TORQUEROD_AH]);
  v.Normalize();
  w = points[LOWERBEAM_C] - points[LOWERBEAM_AH];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_lowerbeam[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_lowerbeam[side]->SetNameString(m_name + "_lowerLink" + suffix);
  m_lowerbeam[side]->SetPos(points[LOWERBEAM_CM]);
  m_lowerbeam[side]->SetRot(rot);
  m_lowerbeam[side]->SetMass(getLowerbeamMass());
  m_lowerbeam[side]->SetInertiaXX(getLowerbeamInertia());
  AddVisualizationLink(m_lowerbeam[side], points[LOWERBEAM_AH], points[LOWERBEAM_C], getLowerbeamRadius(), ChColor(0.2f, 0.6f, 0.2f));
  chassis->GetSystem()->AddBody(m_lowerbeam[side]);
  
  ///*
  ///* ENDOF create and initialize the new bodies of template
  ///*

  ///*
  ///* TODO axle visualisation
  ///* NO HARDPOINT FOR THE AXLE DEFINED --> AXLE NOW at the points[LL_A], points[UL_A] definiton from ChSolidAxle.cpp
  ///* Check it

  // Append to the axle visualization
  AddVisualizationLink(m_axlehousing, points[LOWERBEAM_AH], points[TORQUEROD_AH], getLowerbeamRadius(), ChColor(0.7f, 0.7f, 0.7f));
  ///*
  ///* ENDOF axle visualisation
  ///*


  ///*
  ///* TODO create and initialize the new joints of template
  ///*

  // Create and initialize the revolute joint between axle and knuckle.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction.
  w = points[KNUCKLE_U] - points[KNUCKLE_L];
  w.Normalize();
  u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_revoluteKingpin[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
  m_revoluteKingpin[side]->Initialize(m_axlehousing, m_knuckle[side], ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

  // Create and initialize the spherical joint between axle housing and torque rod.
  m_sphericalTorquerod[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalTorquerod[side]->SetNameString(m_name + "_sphericalTorquerod" + suffix);
  m_sphericalTorquerod[side]->Initialize(m_axlehousing, m_torquerod[side], ChCoordsys<>(points[TORQUEROD_AH], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalTorquerod[side]);

  // Create and initialize the spherical joint between axlehousing and lower beam.
  m_sphericalLowerbeam[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLowerbeam[side]->SetNameString(m_name + "_sphericalLowerbeam" + suffix);
  m_sphericalLowerbeam[side]->Initialize(m_axlehousing, m_lowerbeam[side], ChCoordsys<>(points[LOWERBEAM_AH], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLowerbeam[side]);


  // Create and initialize the universal joint between chassis and torque rod.
  u = dirs[UNIV_TORQUEROD_AXIS_CHASSIS];
  v = dirs[UNIV_TORQUEROD_AXIS_ROD];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);

  m_universalTorquerod[side] = ChSharedPtr<ChLinkUniversal>(new ChLinkUniversal);
  m_universalTorquerod[side]->SetNameString(m_name + "_universalTorquerod" + suffix);
  m_universalTorquerod[side]->Initialize(chassis, m_torquerod[side], ChFrame<>(points[TORQUEROD_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalTorquerod[side]);

  // Create and initialize the universal joint between chassis and lower beam.
  u = dirs[UNIV_LOWERBEAM_AXIS_CHASSIS];
  v = dirs[UNIV_LOWERBEAM_AXIS_BEAM];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);

  m_universalLowerbeam[side] = ChSharedPtr<ChLinkUniversal>(new ChLinkUniversal);
  m_universalLowerbeam[side]->SetNameString(m_name + "_universalLowerbeam" + suffix);
  m_universalLowerbeam[side]->Initialize(chassis, m_lowerbeam[side], ChFrame<>(points[LOWERBEAM_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalLowerbeam[side]);

  ///*
  ///* HH TODO check the following revolute joint
  ///*
  // Create and initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

  m_revolute[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
  m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  ///*
  ///* ENDOF create and initialize the new joints of template
  ///*

  // Create and initialize the spring/damper between axle housing and chassis
  m_shockAH[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_shockAH[side]->SetNameString(m_name + "_shockAH" + suffix);
  m_shockAH[side]->Initialize(chassis, m_axlehousing, false, points[SHOCKAH_C], points[SHOCKAH_AH]);
  m_shockAH[side]->Set_SpringCallback(getShockAHForceCallback());
  chassis->GetSystem()->AddLink(m_shockAH[side]);

  m_springAH[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_springAH[side]->SetNameString(m_name + "_springLB" + suffix);
  m_springAH[side]->Initialize(chassis, m_axlehousing, false, points[SPRINGLB_C], points[SPRINGLB_LB], false, getSpringLBRestLength());
  m_springAH[side]->Set_SpringCallback(getSpringLBForceCallback());
  chassis->GetSystem()->AddLink(m_springLB[side]);

  // Create and initialize the spring/damper between lower beam and chassis
  m_shockLB[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_shockLB[side]->SetNameString(m_name + "_shockLB" + suffix);
  m_shockLB[side]->Initialize(chassis, m_axlehousing, false, points[SHOCKLB_C], points[SHOCKLB_LB]);
  m_shockLB[side]->Set_SpringCallback(getShockLBForceCallback());
  chassis->GetSystem()->AddLink(m_shockLB[side]);

  m_springLB[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_springLB[side]->SetNameString(m_name + "_springLB" + suffix);
  m_springLB[side]->Initialize(chassis, m_axlehousing, false, points[SPRINGLB_C], points[SPRINGLB_LB], false, getSpringLBRestLength());
  m_springLB[side]->Set_SpringCallback(getSpringLBForceCallback());
  chassis->GetSystem()->AddLink(m_springLB[side]);

  // Create and initialize the tierod distance constraint between chassis and upright.
  m_distTierod[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
  m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_K]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);
  
  // Create and initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  m_axle[side] = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle[side]->SetNameString(m_name + "_axle" + suffix);
  m_axle[side]->SetInertia(getAxleInertia());
  chassis->GetSystem()->Add(m_axle[side]);

  m_axle_to_spindle[side] = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
  m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
  chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogHardpointLocations(const ChVector<>& ref,
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
void ChHendricksonPRIMAXX::LogConstraintViolations(ChVehicleSide side)
{
  // Revolute joints
  {
    ChMatrix<>* C = m_revolute[side]->GetC();
    GetLog() << "Spindle revolute      ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  //// TODO
  ///*
  ///* HH QQ: in ChsolidAxle.cpp is a Kingpin revolute
  ///* What is Kingpin
  ///*
  {
    ChMatrix<>* C = m_revoluteKingpin[side]->GetC();
    GetLog() << "Kingpin revolute      ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }
  // Spherical joints
  {
    ChMatrix<>* C = m_sphericalTorquerod[side]->GetC();
    GetLog() << "Torquerod spherical          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_sphericalLowerbeam[side]->GetC();
    GetLog() << "Lowerbeam spherical          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }
  // Universal joints
  {
    ChMatrix<>* C = m_universalTorquerod[side]->GetC();
    GetLog() << "Torquerod universal          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_universalLowerbeam[side]->GetC();
    GetLog() << "Lowerbeam universal          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
  }
  ///* HH thinking about TB constraint
  ///* Put the constraint of TB in here
  ///*

  // Distance constraint
  GetLog() << "Tierod distance       ";
  GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";

}


// -----------------------------------------------------------------------------
//

void ChHendricksonPRIMAXX::AddVisualizationLink(ChSharedBodyPtr   body,
                                                const ChVector<>  pt_1,
                                                const ChVector<>  pt_2,
                                                double            radius,
                                                const ChColor&    color)
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


//-----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                                   double          radius,
                                                   double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}

void ChHendricksonPRIMAXX::AddVisualizationKnuckle(ChSharedBodyPtr   knuckle,
                                                    const ChVector<>  pt_U,
                                                    const ChVector<>  pt_L,
                                                    const ChVector<>  pt_T,
                                                    double            radius)

{
  static const double threshold2 = 1e-6;

  // Express hardpoint locations in body frame.
  ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
  ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
  ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

  if (p_L.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_L(new ChCylinderShape);
    cyl_L->GetCylinderGeometry().p1 = p_L;
    cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_L->GetCylinderGeometry().rad = radius;
    knuckle->AddAsset(cyl_L);
  }

  if (p_U.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
    cyl_U->GetCylinderGeometry().p1 = p_U;
    cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_U->GetCylinderGeometry().rad = radius;
    knuckle->AddAsset(cyl_U);
  }

  if (p_T.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
    cyl_T->GetCylinderGeometry().p1 = p_T;
    cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_T->GetCylinderGeometry().rad = radius;
    knuckle->AddAsset(cyl_T);
  }

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  knuckle->AddAsset(col);
}

} // end namespace chrono
