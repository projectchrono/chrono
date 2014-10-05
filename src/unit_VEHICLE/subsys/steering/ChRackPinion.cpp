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
// Base class for a Rack-Pinion steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The steering link translates along the Y axis. We do not explicitly model the
// pinion but instead use the implied rack-pinion constraint to calculate the
// rack displacement from a given pinion rotation angle.
//
// =============================================================================

#include <vector>

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"
#include "assets/ChTexture.h"

#include "subsys/steering/ChRackPinion.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRackPinion::ChRackPinion(const std::string& name)
: ChSteering(name)
{
  // Create the steering link body
  m_link = ChSharedPtr<ChBody>(new ChBody);
  m_link->SetNameString(name + "_link");

  // Prismatic joint between link and chassis.
  m_prismatic = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic);
  m_prismatic->SetNameString(name + "_prismatic");

  // Linear actuator on link.
  m_actuator = ChSharedPtr<ChLinkLinActuator>(new ChLinkLinActuator);
  m_actuator->SetNameString(name + "_actuator");
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                              const ChVector<>&         location,
                              const ChQuaternion<>&     rotation)
{
  // Express the steering reference frame in the absolute coordinate system.
  ChFrame<> steering_to_abs(location, rotation);
  steering_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Initialize the steering link body
  ChVector<> link_local(0, GetSteeringLinkCOM(), 0);
  ChVector<> link_abs = steering_to_abs.TransformPointLocalToParent(link_local);
  m_link->SetPos(link_abs);
  m_link->SetRot(steering_to_abs.GetRot());
  m_link->SetMass(GetSteeringLinkMass());
  m_link->SetInertiaXX(GetSteeringLinkInertia());
  AddVisualizationSteeringLink();
  chassis->GetSystem()->AddBody(m_link);

  // Initialize the prismatic joint between chassis and link.
  m_prismatic->Initialize(chassis, m_link, ChCoordsys<>(link_abs, steering_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
  chassis->GetSystem()->AddLink(m_prismatic);

  // Initialize the linear actuator.
  // The offset value here must be larger than any possible displacement of the
  // steering link body (the rack) so that we do not reach the singular
  // configuration of the ChLinkLinActuator (when the distance between the two
  // markers becomes zero).
  double offset = 10;
  ChVector<> pt1 = link_abs;
  ChVector<> pt2 = link_abs - offset * steering_to_abs.GetRot().GetYaxis();
  m_actuator->Initialize(chassis, m_link, false, ChCoordsys<>(pt1, QUNIT), ChCoordsys<>(pt2, QUNIT));
  m_actuator->Set_lin_offset(offset);
  chassis->GetSystem()->AddLink(m_actuator);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::Update(double time, double steering)
{
  // Convert the steering input into an angle of the pinion and then into a
  // displacement of the rack.
  double angle = steering * GetMaxAngle();
  double displ = angle * GetPinionRadius();

  if (ChSharedPtr<ChFunction_Const> fun = m_actuator->Get_dist_funct().DynamicCastTo<ChFunction_Const>())
    fun->Set_yconst(displ);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::AddVisualizationSteeringLink()
{
  double length = GetSteeringLinkLength();

  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, length / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -length / 2, 0);
  cyl->GetCylinderGeometry().rad = GetSteeringLinkRadius();
  m_link->AddAsset(cyl);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.8f, 0.8f, 0.2f));
  m_link->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRackPinion::LogConstraintViolations()
{
  // Translational joint
  {
    ChMatrix<>* C = m_prismatic->GetC();
    GetLog() << "Prismatic           ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Actuator
  {
    ChMatrix<>* C = m_actuator->GetC();
    GetLog() << "Actuator            ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
  }

}


}  // end namespace chrono
