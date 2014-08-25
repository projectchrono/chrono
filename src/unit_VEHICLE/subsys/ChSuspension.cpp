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
// Base class for all suspension subsystems
//
// =============================================================================


#include "subsys/ChSuspension.h"


namespace chrono {


ChSuspension::ChSuspension(const std::string& name,
                           Side               side,
                           bool               driven)
: m_name(name),
  m_side(side),
  m_driven(driven)
{
}


void ChSuspension::ApplyAxleTorque(double torque)
{
  assert(m_driven);
  m_axle->SetAppliedTorque(torque);
}


double ChSuspension::GetAxleSpeed() const
{
  assert(m_driven);
  return m_axle->GetPos_dt();
}


void ChSuspension::ApplyTireForce(const ChTireForce& tire_force)
{
  m_spindle->Empty_forces_accumulators();
  m_spindle->Accumulate_force(tire_force.force, tire_force.point, false);
  m_spindle->Accumulate_torque(tire_force.moment, false);
}


}  // end namespace chrono
