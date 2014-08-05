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
// with X pointing towards the rear, Y to the right, and Z up. The origin of
// the reference frame is assumed to be the center of th spindle body (i.e. the
// center of the wheel).
// By default, a right suspension is constructed.  This can be mirrored to
// obtain a left suspension.
// If marked as 'driven', the suspension subsystem also includes an engine link.
//
// =============================================================================

#include "ChDoubleWishboneReduced.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishboneReduced::ChDoubleWishboneReduced(const std::string& name,
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

  // Distance constraints to model upper control arm
  m_distUCA_F = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distUCA_F->SetNameString(name + "_distUCA_F");
  m_distUCA_B = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distUCA_B->SetNameString(name + "_distUCA_B");

  // Distance constraints to model lower control arm
  m_distLCA_F = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distLCA_F->SetNameString(name + "_distLCA_F");
  m_distLCA_B = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distLCA_B->SetNameString(name + "_distLCA_B");

  // Distance constraint to model the tierod
  m_distTierod = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod->SetNameString(name + "_distTierod");

  // Spring-damper
  m_shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_shock->SetNameString(name + "_shock");

  // Engine (only if driven)
  if (m_driven) {
    m_engine = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
    m_engine->SetNameString(name + "_engine");
  }
}


// -----------------------------------------------------------------------------
// TODO: does it make sense to ask for a complete Coordsys (i.e. allow for a 
// non-identity rotation) when attaching a suspension subsystem to the chassis?
// -----------------------------------------------------------------------------
void
ChDoubleWishboneReduced::Initialize(ChSharedBodyPtr   chassis,
                                    const ChVector<>& location)
{
  // Transform all points to absolute frame
  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    if (m_side == LEFT)
      rel_pos.y = -rel_pos.y;
    m_points[i] = chassis->GetCoord().TrasformLocalToParent(location + rel_pos);
  }

  // Set body positions and rotations, mass properties, etc.
  m_spindle->SetPos(m_points[SPINDLE]);
  m_spindle->SetRot(chassis->GetCoord().rot);
  m_spindle->SetMass(getSpindleMass());
  m_spindle->SetInertiaXX(getSpindleInertia());
  OnInitializeSpindle();
  chassis->GetSystem()->AddBody(m_spindle);

  m_upright->SetPos(m_points[UPRIGHT]);
  m_upright->SetRot(chassis->GetCoord().rot);
  m_upright->SetMass(getUprightMass());
  m_upright->SetInertiaXX(getUprightInertia());
  OnInitializeUpright();
  chassis->GetSystem()->AddBody(m_upright);

  // Initialize joints
  ChCoordsys<> rev_csys(m_points[UPRIGHT], Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
  m_revolute->Initialize(m_spindle, m_upright, rev_csys);
  chassis->GetSystem()->AddLink(m_revolute);

  m_distUCA_F->Initialize(chassis, m_upright, false, m_points[UCA_F], m_points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_F);
  m_distUCA_B->Initialize(chassis, m_upright, false, m_points[UCA_B], m_points[UCA_U]);
  chassis->GetSystem()->AddLink(m_distUCA_B);

  m_distLCA_F->Initialize(chassis, m_upright, false, m_points[LCA_F], m_points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_F);
  m_distLCA_B->Initialize(chassis, m_upright, false, m_points[LCA_B], m_points[LCA_U]);
  chassis->GetSystem()->AddLink(m_distLCA_B);

  m_distTierod->Initialize(chassis, m_upright, false, m_points[TIEROD_C], m_points[TIEROD_U]);
  chassis->GetSystem()->AddLink(m_distTierod);

  // Initialize the spring/damper
  m_shock->Initialize(chassis, m_upright, false, m_points[SHOCK_C], m_points[SHOCK_U]);
  m_shock->Set_SpringK(getSpringCoefficient());
  m_shock->Set_SpringR(getDampingCoefficient());
  m_shock->Set_SpringRestLenght(getSpringRestLength());
  chassis->GetSystem()->AddLink(m_shock);

  // Save initial relative position of marker 1 of the tierod distance link,
  // to be used in steering.
  m_tierod_marker = m_distTierod->GetEndPoint1Rel();

  // Initialize the engine link (if driven)
  if (m_driven) {
    ChCoordsys<> eng_csys(m_points[SPINDLE], Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_engine->Initialize(m_spindle, chassis, eng_csys);
    m_engine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO);
    m_engine->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
    chassis->GetSystem()->AddLink(m_engine);
  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double
ChDoubleWishboneReduced::GetSpindleAngSpeed() const
{
  if (m_driven)
    return m_engine->Get_mot_rot_dt();

  return 0;
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void 
ChDoubleWishboneReduced::AttachWheel(ChSharedPtr<ChWheel> wheel)
{
  // Update mass properties of the spindle body
  double wheelMass = wheel->getMass();
  ChVector<> wheelInertia = wheel->getInertia();

  m_spindle->SetMass(m_spindle->GetMass() + wheelMass);
  m_spindle->SetInertiaXX(m_spindle->GetInertiaXX() + wheelInertia);

  // Allow the concrete wheel object to perform any additional initialization
  wheel->OnInitialize(m_spindle);
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::ApplySteering(double displ)
{
  ChVector<> r_bar = m_tierod_marker;
  r_bar.y += displ;
  m_distTierod->SetEndPoint1Rel(r_bar);
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::ApplyTorque(double torque)
{
  assert(m_driven);

  if (ChFunction_Const* efun = dynamic_cast<ChFunction_Const*>(m_engine->Get_tor_funct()))
    efun->Set_yconst(torque);

}


} // end namespace chrono
