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

#include "subsys/ChVehicle.h"


namespace chrono {

  ChVehicle::ChVehicle()
  {
    Set_G_acc(ChVector<>(0, 0, -9.81));

    // Integration and Solver settings
    SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    SetIterLCPmaxItersSpeed(150);
    SetIterLCPmaxItersStab(150);
    SetMaxPenetrationRecoverySpeed(4.0);

  }


  void ChVehicle::Advance(double step)
  {
    DoStepDynamics(step);
  }

}  // end namespace chrono
