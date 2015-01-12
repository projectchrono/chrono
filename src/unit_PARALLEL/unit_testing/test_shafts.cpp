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
// Tests for ChShaft and related components.
//
// =============================================================================

#include "physics/ChShaftsGear.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsInputOutput.h"

using namespace chrono;


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ChSystemParallel* CreateSystem(utils::SystemType sys_type)
{
  // Settings
  int threads = 20;
  bool thread_tuning = false;

  double tolerance = 1e-5;

  int max_iteration_bilateral = 100;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 100;
  int max_iteration_spinning = 0;

  bool clamp_bilaterals = false;
  double bilateral_clamp_speed = 1000;

  double contact_recovery_speed = 1;

  // Create the mechanical system
  ChSystemParallel* system;

  switch (sys_type) {
  case utils::PARALLEL_DEM: system = new ChSystemParallelDEM(); break;
  case utils::PARALLEL_DVI: system = new ChSystemParallelDVI(); break;
  }

  // Set number of threads.
  int max_threads = system->GetParallelThreadNumber();
  if (threads > max_threads) threads = max_threads;
  system->SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);

  system->GetSettings()->max_threads = threads;
  system->GetSettings()->perform_thread_tuning = thread_tuning;

  // Edit system settings
  system->GetSettings()->solver.tolerance = tolerance;
  system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
  system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
  system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

  if (sys_type == utils::PARALLEL_DVI) {
    ChSystemParallelDVI* systemDVI = static_cast<ChSystemParallelDVI*>(system);
    systemDVI->GetSettings()->solver.solver_mode = SLIDING;
    systemDVI->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    systemDVI->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    systemDVI->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    systemDVI->ChangeSolverType(APGD);
  }

  return system;
}

bool TestShaftShaft(utils::SystemType sys_type)
{
  // Create the system
  ChSystemParallel* system = CreateSystem(sys_type);

  // Create two 1-D shaft objects, with a constant torque applied to the first
  // shaft. By default, a ChShaft is free to rotate.
  ChSharedPtr<ChShaft> shaftA(new ChShaft);
  shaftA->SetInertia(10);
  shaftA->SetAppliedTorque(6);
  system->Add(shaftA);

  ChSharedPtr<ChShaft> shaftB(new ChShaft);
  shaftB->SetInertia(100);
  system->Add(shaftB);

  // Create a connection between the two shafts with a given trnsmission ratio.
  ChSharedPtr<ChShaftsGear> gearAB(new ChShaftsGear);
  gearAB->Initialize(shaftA, shaftB);
  gearAB->SetTransmissionRatio(-0.1);
  system->Add(gearAB);

  // Perform the simulation.
  bool passed = true;
  double time_end = 5;
  double time_step = 1e-3;
  double time = 0;

  while (time < time_end)
  {
    system->DoStepDynamics(time_step);
    time += time_step;
  }

  std::cout << "Time: " << time << "\n"
    << "  shaft A rot: " << shaftA->GetPos()
    << "  speed: " << shaftA->GetPos_dt()
    << "  accel: " << shaftA->GetPos_dtdt()
    << "\n"
    << "  shaft B rot: " << shaftB->GetPos()
    << "  speed: " << shaftB->GetPos_dt()
    << "  accel: " << shaftB->GetPos_dtdt()
    << "\n"
    << "  torque on A side: " << gearAB->GetTorqueReactionOn1()
    << "  torque on B side: " << gearAB->GetTorqueReactionOn2()
    << "\n";

  // Delete the system
  delete system;

  return passed;
}


// -----------------------------------------------------------------------------
// Main driver function for running the simulation and validating the results.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  bool test_passed = true;

  // Test shaft - shaft connection
  test_passed &= TestShaftShaft(utils::PARALLEL_DEM);
  test_passed &= TestShaftShaft(utils::PARALLEL_DVI);

  // Test shaft - body connection
  ////test_passed &= TestShaftBody(utils::PARALLEL_DEM);
  ////test_passed &= TestShaftBody(utils::PARALLEL_DVI);


  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}
