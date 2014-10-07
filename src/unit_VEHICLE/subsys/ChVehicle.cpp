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
// Base class for a vehicle model.
//
// =============================================================================

#include <algorithm>

#include "subsys/ChVehicle.h"
#include "subsys/ChDriveline.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Constructor for a ChVehicle: specify default step size and solver parameters.
// -----------------------------------------------------------------------------
ChVehicle::ChVehicle()
: m_stepsize(1e-3)
{
  Set_G_acc(ChVector<>(0, 0, -9.81));

  // Integration and Solver settings
  SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  SetIterLCPmaxItersSpeed(150);
  SetIterLCPmaxItersStab(150);
  SetMaxPenetrationRecoverySpeed(4.0);
}


// -----------------------------------------------------------------------------
// Advance the state of the system, taking as many steps as needed to exactly
// reach the specified value 'step'.
// ---------------------------------------------------------------------------- -
void ChVehicle::Advance(double step)
{
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    DoStepDynamics(h);
    t += h;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSharedPtr<ChBody> ChVehicle::GetWheelBody(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindle(wheel_id.side());
}

const ChVector<>& ChVehicle::GetWheelPos(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindlePos(wheel_id.side());
}

const ChQuaternion<>& ChVehicle::GetWheelRot(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindleRot(wheel_id.side());
}

const ChVector<>& ChVehicle::GetWheelLinVel(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindleLinVel(wheel_id.side());
}

ChVector<> ChVehicle::GetWheelAngVel(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindleAngVel(wheel_id.side());
}

double ChVehicle::GetWheelOmega(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetAxleSpeed(wheel_id.side());
}


// -----------------------------------------------------------------------------
// Return the complete state (expressed in the global frame) for the specified
// wheel body.
// -----------------------------------------------------------------------------
ChWheelState ChVehicle::GetWheelState(const ChWheelID& wheel_id) const
{
  ChWheelState state;

  state.pos = GetWheelPos(wheel_id);
  state.rot = GetWheelRot(wheel_id);
  state.lin_vel = GetWheelLinVel(wheel_id);
  state.ang_vel = GetWheelAngVel(wheel_id);

  ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
  state.omega = ang_vel_loc.y;

  return state;
}

// -----------------------------------------------------------------------------
// Return the global driver position
// -----------------------------------------------------------------------------
ChVector<> ChVehicle::GetDriverPos() const
{
  return m_chassis->GetCoord().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChSharedPtr<ChShaft> ChVehicle::GetDriveshaft() const
{
  return m_driveline->GetDriveshaft();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChVehicle::GetDriveshaftSpeed() const
{
  return m_driveline->GetDriveshaftSpeed();
}


// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChVehicle::LogConstraintViolations()
{
  GetLog().SetNumFormat("%16.4e");

  // Report constraint violations for the suspension joints
  for (size_t i = 0; i < m_suspensions.size(); i++) {
    GetLog() << "\n---- AXLE " << i << " LEFT side suspension constraint violations\n\n";
    m_suspensions[i]->LogConstraintViolations(LEFT);
    GetLog() << "\n---- AXLE " << i << " RIGHT side suspension constraint violations\n\n";
    m_suspensions[i]->LogConstraintViolations(RIGHT);
  }

  // Report constraint violations for the steering joints
  GetLog() << "\n---- STEERING constrain violations\n\n";
  m_steering->LogConstraintViolations();

  GetLog().SetNumFormat("%g");

}


}  // end namespace chrono
