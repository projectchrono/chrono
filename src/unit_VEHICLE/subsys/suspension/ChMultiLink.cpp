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

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "subsys/suspension/ChMultiLink.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMultiLink::m_pointNames[] = {
    "SPINDLE  ",
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
    "TIEROD_U "
};


// -----------------------------------------------------------------------------
// Default shock and spring functors (used for linear elements)
// -----------------------------------------------------------------------------
class LinearSpringForce : public ChSpringForceCallback
{
public:
  LinearSpringForce(double k) : m_k(k) {}

  virtual double operator()(double time,         // current time
                            double rest_length,  // undeformed length
                            double length,       // current length
                            double vel)          // current velocity (positive when extending)
  {
    return -m_k * (length - rest_length);
  }

private:
  double  m_k;
};

class LinearShockForce : public ChSpringForceCallback
{
public:
  LinearShockForce(double c) : m_c(c) {}

  virtual double operator()(double time,         // current time
                            double rest_length,  // undeformed length
                            double length,       // current length
                            double vel)          // current velocity (positive when extending)
  {
    return -m_c * vel;
  }

private:
  double  m_c;
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMultiLink::ChMultiLink(const std::string& name)
: ChSuspension(name),
  m_nonlinearShock(false),
  m_nonlinearSpring(false),
  m_shockCB(NULL),
  m_springCB(NULL)
{
  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

ChMultiLink::~ChMultiLink()
{
  // If we own the shock or spring callbacks, delete them.
  // Note: this is the only reason why the booleans m_nonlinearShock and
  // m_nonlinearSpring had to be cashed (we cannot call a virtual method in the
  // destructor of the base class).

  if (!m_nonlinearShock)
    delete m_shockCB;

  if (!m_nonlinearSpring)
    delete m_springCB;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::CreateSide(ChVehicleSide      side,
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

  // Bodies and constraints to model upper arm
  m_upperArm[side] = ChSharedBodyPtr(new ChBody);
  m_upperArm[side]->SetNameString(m_name + "_upperArm" + suffix);
  m_revoluteUA[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteUA[side]->SetNameString(m_name + "_revoluteUA" + suffix);
  m_sphericalUA[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUA[side]->SetNameString(m_name + "_sphericalUA" + suffix);

  // Bodies and constraints to model lateral
  m_lateral[side] = ChSharedBodyPtr(new ChBody);
  m_lateral[side]->SetNameString(m_name + "_lateral" + suffix);
  m_universalLateralChassis[side] = ChSharedPtr<ChLinkLockUniversal>(new ChLinkLockUniversal);
  m_universalLateralChassis[side]->SetNameString(m_name + "_universalLateralChassis" + suffix);
  m_sphericalLateralUpright[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLateralUpright[side]->SetNameString(m_name + "_sphericalLateralUpright" + suffix);

  // Bodies and constraints to model trailing link
  m_trailingLink[side] = ChSharedBodyPtr(new ChBody);
  m_trailingLink[side]->SetNameString(m_name + "_trailingLink" + suffix);
  m_universalTLChassis[side] = ChSharedPtr<ChLinkLockUniversal>(new ChLinkLockUniversal);
  m_universalTLChassis[side]->SetNameString(m_name + "_universalTLChassis" + suffix);
  m_sphericalTLUpright[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalTLUpright[side]->SetNameString(m_name + "_sphericalTLUpright" + suffix);

  // Distance constraint to model the tierod
  m_distTierod[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);

  // Spring-damper
  m_shock[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_shock[side]->SetNameString(m_name + "_shock" + suffix);
  m_spring[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_spring[side]->SetNameString(m_name + "_spring" + suffix);

  // Create the axle shaft and its connection to the spindle.
  m_axle[side] = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle[side]->SetNameString(m_name + "_axle" + suffix);
  m_axle_to_spindle[side] = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                             const ChVector<>&          location,
                             ChSharedPtr<ChBody>        tierod_body)
{
  // Set the shock and spring force callbacks (use the user-provided functor if
  // a nonlinear element was specified; otherwise, use the default functor).
  m_nonlinearShock = useNonlinearShock();
  m_nonlinearSpring = useNonlinearSpring();
  m_shockCB = m_nonlinearShock ? getShockForceCallback() : new LinearShockForce(getDampingCoefficient());
  m_springCB = m_nonlinearSpring ? getSpringForceCallback() : new LinearSpringForce(getSpringCoefficient());

  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Transform all points to absolute frame and initialize left side.
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

void ChMultiLink::InitializeSide(ChVehicleSide                   side,
                                 ChSharedPtr<ChBodyAuxRef>       chassis,
                                 ChSharedPtr<ChBody>             tierod_body,
                                 const std::vector<ChVector<> >& points,
                                 const std::vector<ChVector<> >& dirs)
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
  AddVisualizationUpright(m_upright[side], points[UA_U], points[LAT_U], points[TL_U], points[TIEROD_U], points[UPRIGHT], getUprightRadius());
  chassis->GetSystem()->AddBody(m_upright[side]);

  // Unit vectors for orientation matrices.
  ChVector<> u;
  ChVector<> v;
  ChVector<> w;
  ChMatrix33<> rot;

  // Initialize Upper Arm body.
  // Determine the rotation matrix of the upper arm based on the plane of the hard points
  // (z axis normal to the plane of the upper arm)
  w = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
  w.Normalize();
  u = points[UA_F] - points[UA_B];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_upperArm[side]->SetPos(points[UA_CM]);
  m_upperArm[side]->SetRot(rot);
  m_upperArm[side]->SetMass(getUpperArmMass());
  m_upperArm[side]->SetInertiaXX(getUpperArmInertia());
  AddVisualizationUpperArm(m_upperArm[side], points[UA_F], points[UA_B], points[UA_U], getUpperArmRadius());
  chassis->GetSystem()->AddBody(m_upperArm[side]);

  // Initialize lateral body.
  // Determine the rotation matrix of the lateral based on the plane of the hard points
  // (z-axis along the length of the track rod)
  v = Vcross(points[LAT_U] - points[TL_U], points[LAT_C] - points[TL_U]);
  v.Normalize();
  w = points[LAT_C] - points[LAT_U];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_lateral[side]->SetPos(points[LAT_CM]);
  m_lateral[side]->SetRot(rot);
  m_lateral[side]->SetMass(getLateralMass());
  m_lateral[side]->SetInertiaXX(getLateralInertia());
  AddVisualizationLateral(m_lateral[side], points[LAT_U], points[LAT_C], getLateralRadius());
  chassis->GetSystem()->AddBody(m_lateral[side]);

  // Initialize trailing link body.
  // Determine the rotation matrix of the trailing link based on the plane of the hard points
  // (z-axis along the length of the trailing link)
  v = Vcross(points[TL_U] - points[SPRING_L], points[TL_C] - points[SPRING_L]);
  v.Normalize();
  w = points[TL_C] - points[TL_U];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_trailingLink[side]->SetPos(points[TL_CM]);
  m_trailingLink[side]->SetRot(rot);
  m_trailingLink[side]->SetMass(getTrailingLinkMass());
  m_trailingLink[side]->SetInertiaXX(getTrailingLinkInertia());
  AddVisualizationTrailingLink(m_trailingLink[side], points[TL_C], points[SPRING_L], points[TL_U], getTrailingLinkRadius());
  chassis->GetSystem()->AddBody(m_trailingLink[side]);

  // Initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  // Initialize the revolute joint between chassis and upper arm.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the upper arm.
  v = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
  v.Normalize();
  w = points[UA_F] - points[UA_B];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteUA[side]->Initialize(chassis, m_upperArm[side], ChCoordsys<>((points[UA_F]+points[UA_B])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteUA[side]);

  // Initialize the spherical joint between upright and upper arm.
  m_sphericalUA[side]->Initialize(m_upperArm[side], m_upright[side], ChCoordsys<>(points[UA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUA[side]);

  // Initialize the spherical joint between upright and track rod.
  m_sphericalLateralUpright[side]->Initialize(m_lateral[side], m_upright[side], ChCoordsys<>(points[LAT_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLateralUpright[side]);

  // Initialize the universal joint between chassis and track rod.
  u = dirs[UNIV_AXIS_CHASSIS_LAT];
  v = dirs[UNIV_AXIS_LINK_LAT];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);
  m_universalLateralChassis[side]->Initialize(m_lateral[side], chassis, ChCoordsys<>(points[LAT_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalLateralChassis[side]);

  // Initialize the spherical joint between upright and trailing link.
  m_sphericalTLUpright[side]->Initialize(m_trailingLink[side], m_upright[side], ChCoordsys<>(points[TL_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalTLUpright[side]);

  // Initialize the universal joint between chassis and trailing link.
  u = dirs[UNIV_AXIS_CHASSIS_TL];
  v = dirs[UNIV_AXIS_LINK_TL];
  w = Vcross(u, v);
  rot.Set_A_axis(u, v, w);
  m_universalTLChassis[side]->Initialize(m_trailingLink[side], chassis, ChCoordsys<>(points[TL_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_universalTLChassis[side]);

  // Initialize the tierod distance constraint between chassis and upright.
  m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_trailingLink[side], false, points[SHOCK_C], points[SHOCK_L]);
  m_shock[side]->Set_SpringCallback(m_shockCB);
  chassis->GetSystem()->AddLink(m_shock[side]);

  m_spring[side]->Initialize(chassis, m_trailingLink[side], false, points[SPRING_C], points[SPRING_L], false, getSpringRestLength());
  m_spring[side]->Set_SpringCallback(m_springCB);
  chassis->GetSystem()->AddLink(m_spring[side]);

  // Initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  m_axle[side]->SetInertia(getAxleInertia());
  chassis->GetSystem()->Add(m_axle[side]);

  m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
  chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogHardpointLocations(const ChVector<>& ref,
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
void ChMultiLink::LogConstraintViolations(ChVehicleSide side)
{
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
void ChMultiLink::AddVisualizationUpperArm(ChSharedBodyPtr   arm,
                                           const ChVector<>  pt_F,
                                           const ChVector<>  pt_B,
                                           const ChVector<>  pt_U,
                                           double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = arm->TransformPointParentToLocal(pt_F);
  ChVector<> p_B = arm->TransformPointParentToLocal(pt_B);
  ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.6f, 0.2f, 0.6f));
  arm->AddAsset(col);
}

void ChMultiLink::AddVisualizationUpright(ChSharedBodyPtr   upright,
                                          const ChVector<>  pt_UA,
                                          const ChVector<>  pt_TR,
                                          const ChVector<>  pt_TL,
                                          const ChVector<>  pt_T,
                                          const ChVector<>  pt_U,
                                          double            radius)
{
  static const double threshold2 = 1e-6;

  // Express hardpoint locations in body frame.
  ChVector<> p_UA = upright->TransformPointParentToLocal(pt_UA);
  ChVector<> p_TR = upright->TransformPointParentToLocal(pt_TR);
  ChVector<> p_TL = upright->TransformPointParentToLocal(pt_TL);
  ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);
  ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);

  if (p_UA.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_UA(new ChCylinderShape);
    cyl_UA->GetCylinderGeometry().p1 = p_UA;
    cyl_UA->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_UA->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_UA);
  }

  if (p_TR.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_TR(new ChCylinderShape);
    cyl_TR->GetCylinderGeometry().p1 = p_TR;
    cyl_TR->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_TR->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_TR);
  }

  if (p_TL.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_TL(new ChCylinderShape);
    cyl_TL->GetCylinderGeometry().p1 = p_TL;
    cyl_TL->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_TL->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_TL);
  }

  if (p_T.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
    cyl_T->GetCylinderGeometry().p1 = p_T;
    cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_T->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_T);
  }

  if (p_U.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
    cyl_U->GetCylinderGeometry().p1 = p_U;
    cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_U->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_U);
  }

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  upright->AddAsset(col);
}

void ChMultiLink::AddVisualizationLateral(ChSharedBodyPtr   rod,
                                          const ChVector<>  pt_C,
                                          const ChVector<>  pt_U,
                                          double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_C = rod->TransformPointParentToLocal(pt_C);
  ChVector<> p_U = rod->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = p_C;
  cyl->GetCylinderGeometry().p2 = p_U;
  cyl->GetCylinderGeometry().rad = radius;
  rod->AddAsset(cyl);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.6f, 0.2f));
  rod->AddAsset(col);
}

void ChMultiLink::AddVisualizationTrailingLink(ChSharedBodyPtr   link,
                                               const ChVector<>  pt_C,
                                               const ChVector<>  pt_S,
                                               const ChVector<>  pt_U,
                                               double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_C = link->TransformPointParentToLocal(pt_C);
  ChVector<> p_S = link->TransformPointParentToLocal(pt_S);
  ChVector<> p_U = link->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl1(new ChCylinderShape);
  cyl1->GetCylinderGeometry().p1 = p_C;
  cyl1->GetCylinderGeometry().p2 = p_S;
  cyl1->GetCylinderGeometry().rad = radius;
  link->AddAsset(cyl1);

  ChSharedPtr<ChCylinderShape> cyl2(new ChCylinderShape);
  cyl2->GetCylinderGeometry().p1 = p_S;
  cyl2->GetCylinderGeometry().p2 = p_U;
  cyl2->GetCylinderGeometry().rad = radius;
  link->AddAsset(cyl2);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.6f, 0.6f));
  link->AddAsset(col);
}


void ChMultiLink::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                          double          radius,
                                          double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}


} // end namespace chrono
