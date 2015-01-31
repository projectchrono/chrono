// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// Base class for a Track vehicle model.
//
// =============================================================================

#include <algorithm>

#include "ChTrackVehicle.h"


namespace chrono {


ChTrackVehicle::ChTrackVehicle()
: m_ownsSystem(true),
  m_stepsize(1e-3),
  m_num_engines(0)
{
  m_system = new ChSystem;

  m_system->Set_G_acc(ChVector<>(0, -9.81, 0));

  // Integration and Solver settings
  m_system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  m_system->SetIterLCPmaxItersSpeed(150);
  m_system->SetIterLCPmaxItersStab(150);
  m_system->SetMaxPenetrationRecoverySpeed(2.0);
  // SetMaxPenetrationRecoverySpeed(2.0);
  // SetIterLCPomega(2.0);
  // SetIterLCPsharpnessLambda(2.0);
}


ChTrackVehicle::ChTrackVehicle(ChSystem* system)
: m_system(system),
  m_ownsSystem(false),
  m_stepsize(1e-3),
  m_num_engines(0)
{
}


ChTrackVehicle::~ChTrackVehicle()
{
  if (m_ownsSystem)
    delete m_system;
}


// -----------------------------------------------------------------------------
// Advance the state of the system, taking as many steps as needed to exactly
// reach the specified value 'step'.
void ChTrackVehicle::Advance(double step)
{
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    m_system->DoStepDynamics(h);
    t += h;
  }
}

// -----------------------------------------------------------------------------
// Return the global driver position
// -----------------------------------------------------------------------------
ChVector<> ChTrackVehicle::GetDriverPos() const
{
  return m_chassis->GetCoord().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}

}  // end namespace chrono
