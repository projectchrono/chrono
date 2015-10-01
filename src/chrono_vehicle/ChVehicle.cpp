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

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriveline.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Constructor for a ChVehicle using a default Chrono Chsystem.
// Specify default step size and solver parameters.
// -----------------------------------------------------------------------------
ChVehicle::ChVehicle()
: m_ownsSystem(true),
  m_stepsize(1e-3)
{
  m_system = new ChSystem;

  m_system->Set_G_acc(ChVector<>(0, 0, -9.81));

  // Integration and Solver settings
  m_system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  m_system->SetIterLCPmaxItersSpeed(150);
  m_system->SetIterLCPmaxItersStab(150);
  m_system->SetMaxPenetrationRecoverySpeed(4.0);
}


// -----------------------------------------------------------------------------
// Constructor for a ChVehicle using a default Chrono ChSystem.
// -----------------------------------------------------------------------------
ChVehicle::ChVehicle(ChSystem* system)
: m_system(system),
  m_ownsSystem(false),
  m_stepsize(1e-3)
{
}


// -----------------------------------------------------------------------------
// Destructor for ChVehicle
// -----------------------------------------------------------------------------
ChVehicle::~ChVehicle()
{
  if (m_ownsSystem)
    delete m_system;
}


// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between
// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
// from the powertrain, and tire forces (expressed in the global reference
// frame).
// The default implementation of this function invokes the update functions for
// all vehicle subsystems.
// -----------------------------------------------------------------------------
void ChVehicle::Update(double              time,
                       double              steering,
                       double              braking,
                       double              powertrain_torque,
                       const ChTireForces& tire_forces)
{
  // Apply powertrain torque to the driveline's input shaft.
  m_driveline->Update(powertrain_torque);

  // Let the steering subsystems process the steering input.
  for (unsigned int i = 0; i < m_steerings.size(); i++) {
    m_steerings[i]->Update(time, steering);
  }

  // Apply tire forces to spindle bodies and apply braking.
  for (unsigned int i = 0; i < m_suspensions.size(); i++) {
    m_suspensions[i]->Update(LEFT, tire_forces[2 * i]);
    m_suspensions[i]->Update(RIGHT, tire_forces[2 * i + 1]);

    m_brakes[2 * i]->Update(braking);
    m_brakes[2 * i + 1]->Update(braking);
  }
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
    m_system->DoStepDynamics(h);
    t += h;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChVector<> ChVehicle::GetVehicleAcceleration(const ChVector<>& locpos) const {
    ChVector<> acc_abs = m_chassis->GetFrame_REF_to_abs().PointAccelerationLocalToParent(locpos);
    return m_chassis->GetFrame_REF_to_abs().TransformDirectionParentToLocal(acc_abs);
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
  for (size_t i = 0; i < m_steerings.size(); i++) {
    GetLog() << "\n---- STEERING subsystem " << i << " constraint violations\n\n";
    m_steerings[i]->LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");

}


}  // end namespace chrono
