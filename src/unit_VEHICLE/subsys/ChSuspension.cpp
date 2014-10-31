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


ChSuspension::ChSuspension(const std::string& name)
: m_name(name)
{
}


void ChSuspension::ApplyAxleTorque(ChVehicleSide side,
                                   double        torque)
{
  m_axle[side]->SetAppliedTorque(torque);
}


void ChSuspension::ApplyTireForce(ChVehicleSide      side,
                                  const ChTireForce& tire_force)
{
  m_spindle[side]->Empty_forces_accumulators();
  m_spindle[side]->Accumulate_force(tire_force.force, tire_force.point, false);
  m_spindle[side]->Accumulate_torque(tire_force.moment, false);
}


}  // end namespace chrono
