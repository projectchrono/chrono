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
ChDoubleWishbone::ChDoubleWishbone(const std::string& name,
                                   bool               steerable,
                                   bool               driven)
: ChSuspension(name, steerable, driven),
  m_nonlinearShock(false),
  m_nonlinearSpring(false),
  m_shockCB(NULL),
  m_springCB(NULL)
{
  CreateSide(LEFT, "_L");
  CreateSide(RIGHT, "_R");
}

ChDoubleWishbone::~ChDoubleWishbone()
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
  m_shock[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_shock[side]->SetNameString(m_name + "_shock" + suffix);
  m_spring[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
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
void ChDoubleWishbone::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
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

void ChDoubleWishbone::InitializeSide(ChSuspension::Side              side,
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

  // Unit vectors for orientation matrices.
  ChVector<> u;
  ChVector<> v;
  ChVector<> w;
  ChMatrix33<> rot;

  // Initialize Upper Control Arm body.
  // Determine the rotation matrix of the UCA based on the plane of the hard points
  // (z axis normal to the plane of the UCA)
  w = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
  w.Normalize();
  u = points[UCA_F] - points[UCA_B];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_UCA[side]->SetPos(points[UCA_CM]);
  m_UCA[side]->SetRot(rot);
  m_UCA[side]->SetMass(getUCAMass());
  m_UCA[side]->SetInertiaXX(getUCAInertia());
  AddVisualizationControlArm(m_UCA[side], points[UCA_F], points[UCA_B], points[UCA_U], getUCARadius());
  chassis->GetSystem()->AddBody(m_UCA[side]);

  // Initialize Lower Control Arm body.
  // Determine the rotation matrix of the LCA, based on the plane of the hard points
  // (z axis normal to the plane of the LCA)
  w = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
  w.Normalize();
  u = points[LCA_F] - points[LCA_B];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_LCA[side]->SetPos(points[LCA_CM]);
  m_LCA[side]->SetRot(rot);
  m_LCA[side]->SetMass(getLCAMass());
  m_LCA[side]->SetInertiaXX(getLCAInertia());
  AddVisualizationControlArm(m_LCA[side], points[LCA_F], points[LCA_B], points[LCA_U], getLCARadius());
  chassis->GetSystem()->AddBody(m_LCA[side]);


  // Initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys((points[UPRIGHT] + points[SPINDLE]) / 2, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  // Initialize the revolute joint between chassis and UCA.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the UCA.
  v = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
  v.Normalize();
  w = points[UCA_F] - points[UCA_B];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteUCA[side]->Initialize(chassis, m_UCA[side], ChCoordsys<>((points[UCA_F]+points[UCA_B])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteUCA[side]);

  // Initialize the spherical joint between upright and UCA.
  m_sphericalUCA[side]->Initialize(m_UCA[side], m_upright[side], ChCoordsys<>(points[UCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUCA[side]);

  // Initialize the revolute joint between chassis and LCA.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the LCA.
  v = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
  v.Normalize();
  w = points[LCA_F] - points[LCA_B];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteLCA[side]->Initialize(chassis, m_LCA[side], ChCoordsys<>((points[LCA_F]+points[LCA_B])/2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteLCA[side]);

  // Initialize the spherical joint between upright and LCA.
  m_sphericalLCA[side]->Initialize(m_LCA[side], m_upright[side], ChCoordsys<>(points[LCA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLCA[side]);

  // Initialize the tierod distance constraint between chassis and upright.
  m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Initialize the spring/damper
  m_shock[side]->Initialize(chassis, m_LCA[side], false, points[SHOCK_C], points[SHOCK_A]);
  m_shock[side]->Set_SpringCallback(m_shockCB);
  chassis->GetSystem()->AddLink(m_shock[side]);

  m_spring[side]->Initialize(chassis, m_LCA[side], false, points[SPRING_C], points[SPRING_A], false, getSpringRestLength());
  m_spring[side]->Set_SpringCallback(m_springCB);
  chassis->GetSystem()->AddLink(m_spring[side]);

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
    GetLog() << "LCA revolute          ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_revoluteUCA[side]->GetC();
    GetLog() << "UCA revolute          ";
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
  {
    ChMatrix<>* C = m_sphericalUCA[side]->GetC();
    GetLog() << "UCA spherical         ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }

  // Distance constraint
  GetLog() << "Tierod distance       ";
  GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishbone::AddVisualizationControlArm(ChSharedBodyPtr    arm,
                                                  const ChVector<>&  pt_F,
                                                  const ChVector<>&  pt_B,
                                                  const ChVector<>&  pt_U,
                                                  double             radius)
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
  col->SetColor(ChColor(0.7f, 0.7f, 0.7f));
  arm->AddAsset(col);
}


void ChDoubleWishbone::AddVisualizationUpright(ChSharedBodyPtr    upright,
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

void ChDoubleWishbone::AddVisualizationSpindle(ChSharedBodyPtr spindle,
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
