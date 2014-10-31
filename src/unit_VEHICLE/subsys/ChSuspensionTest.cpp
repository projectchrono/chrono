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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Base class for a suspension testing mechanism, includes two suspension units
// and steering attached to a chassis which is fixed to ground, and wheels that
// are locked (e.g., do not rotate, but can apply apply test forces for actuation)
//
// =============================================================================

#include <algorithm>

#include "subsys/ChSuspensionTest.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Constructor for a ChSuspension: specify default step size and solver parameters.
// -----------------------------------------------------------------------------
ChSuspension::ChSuspension()
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
void ChSuspension::Advance(double step)
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
ChSharedPtr<ChBody> ChSuspension::GetWheelBody(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindle(wheel_id.side());
}

const ChVector<>& ChSuspension::GetWheelPos(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindlePos(wheel_id.side());
}

const ChVector<>& ChSuspension::GetWheelLinVel(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindleLinVel(wheel_id.side());
}

// -----------------------------------------------------------------------------
// Return the global driver position
// -----------------------------------------------------------------------------
ChVector<> ChSuspension::GetDriverPos() const
{
  return m_chassis->GetCoord().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}


// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChSuspension::LogConstraintViolations()
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
