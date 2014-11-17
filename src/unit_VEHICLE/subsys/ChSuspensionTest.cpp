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
// Basically, a ChVehicle without rear subsystems nor braking/throttle
// =============================================================================

#include <algorithm>

#include "subsys/ChSuspensionTest.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Constructor for a ChSuspensiontester: specify default step size and solver parameters.
// -----------------------------------------------------------------------------
ChSuspensionTest::ChSuspensionTest()
: m_stepsize(1e-3), m_has_steering(false)
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
void ChSuspensionTest::Advance(double step)
{
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    DoStepDynamics(h);
    t += h;
  }
}


// -----------------------------------------------------------------------------
// Accessors
ChSharedPtr<ChBody> ChSuspensionTest::GetWheelBody(const ChWheelID& wheel_id) const
{
  return m_suspension->GetSpindle(wheel_id.side());
}

const ChVector<>& ChSuspensionTest::GetWheelPos(const ChWheelID& wheel_id) const
{
  return m_suspension->GetSpindlePos(wheel_id.side());
}

const ChQuaternion<>& ChSuspensionTest::GetWheelRot(const ChWheelID& wheel_id) const
{
  return m_suspension->GetSpindleRot(wheel_id.side());
}

const ChVector<>& ChSuspensionTest::GetWheelLinVel(const ChWheelID& wheel_id) const
{
  return m_suspension->GetSpindleLinVel(wheel_id.side());
}

const ChVector<>& ChSuspensionTest::GetWheelAngVel(const ChWheelID& wheel_id) const
{
  return m_suspension->GetSpindleAngVel(wheel_id.side());
}

ChWheelState ChSuspensionTest::GetWheelState(const ChWheelID& wheel_id) const
{
  ChWheelState state;

  state.pos = GetWheelPos(wheel_id);
  state.rot = GetWheelRot(wheel_id);
  state.lin_vel = GetWheelLinVel(wheel_id);
  state.ang_vel = 0;

  ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
  state.omega = ang_vel_loc.y;

  return state;
}

ChVector<> ChSuspensionTest::GetDriverPos() const
{
  return m_ground->GetCoord().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}


// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChSuspensionTest::LogConstraintViolations()
{
  GetLog().SetNumFormat("%16.4e");

  // Report constraint violations for the suspension joints
  GetLog() << "\n---- LEFT side suspension constraint violations\n\n";
  m_suspension->LogConstraintViolations(LEFT);
  GetLog() << "\n---- RIGHT side suspension constraint violations\n\n";
  m_suspension->LogConstraintViolations(RIGHT);

  // Report constraint violations for the steering joints
  GetLog() << "\n---- STEERING constrain violations\n\n";
  m_steering->LogConstraintViolations();

  GetLog().SetNumFormat("%g");
}


}  // end namespace chrono
