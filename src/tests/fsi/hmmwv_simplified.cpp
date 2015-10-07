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
// Author: Radu Serban
// =============================================================================
//
// Chrono::Vehicle + ChronoParallel demo program for simulating a HMMWV mVehicle
// over rigid or granular material.
//
// The mVehicle model uses the utility class ChWheeledVehicleAssembly and is
// based on JSON specification files from the Chrono data directory.
//
// Contact uses the DEM-C (complementarity) formulation.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"
#include "utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChWheeledVehicleAssembly.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChCNarrowphaseRUtils.h"

#include "VehicleExtraProperties.h"
#include "fsi_hmmwv_params.h"

// Control use of OpenGL run-time rendering
// Note: CHRONO_OPENGL is defined in ChConfigParallel.h
//#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "demo_utils.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Define General variables
SimParams paramsH;

// =============================================================================

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Duration of the "hold time" (mVehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.

double tolerance = 0.1;

int max_iteration_bilateral = 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 2000;
int max_iteration_spinning = 0;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// =============================================================================

void DoStepChronoSystem(ChSystemParallelDVI* mphysicalSystem, double dT, double mTime) {
  // Release the mVehicle chassis at the end of the hold time.
  if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed() && mTime > time_hold_vehicle) {
    mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
    for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
      mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
    }
  }

  // Update mVehicle
  mVehicle->Update(mTime);

#if irrlichtVisualization
  Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
  if (!(application->GetDevice()->run()))
    return 0;
  application->SetTimestep(dT);
  application->GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));
  ChIrrTools::drawGrid(application->GetVideoDriver(),
                       2 * paramsH.HSML,
                       2 * paramsH.HSML,
                       50,
                       50,
                       ChCoordsys<>(ChVector<>(domainCenter.x, paramsH.worldOrigin.y, domainCenter.z),
                                    Q_from_AngAxis(CH_C_PI / 2, VECT_X)),
                       video::SColor(50, 90, 90, 150),
                       true);
  application->DrawAll();
  application->DoStep();
  application->GetVideoDriver()->endScene();
#else
#ifdef CHRONO_OPENGL
  if (gl_window.Active()) {
    gl_window.DoStepDynamics(dT);
    gl_window.Render();
  }
#else
  mphysicalSystem->DoStepDynamics(dT);
#endif
#endif
}

// =============================================================================

void InitializeChronoGraphics(ChSystemParallelDVI* mphysicalSystem) {
  //	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
  //	ChVector<> CameraLocation = ChVector<>(2 * paramsH.cMax.x, 2 * paramsH.cMax.y, 2 * paramsH.cMax.z);
  //	ChVector<> CameraLookAt = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);
  ChVector<> CameraLocation = ChVector<>(0, -10, 0);
  ChVector<> CameraLookAt = ChVector<>(0, 0, 0);

#ifdef CHRONO_OPENGL
  gl_window.Initialize(1280, 720, "HMMWV", mphysicalSystem);
  gl_window.SetCamera(CameraLocation, CameraLookAt, ChVector<>(0, 0, 1));
  gl_window.SetRenderMode(opengl::WIREFRAME);

// Uncomment the following two lines for the OpenGL manager to automatically un the simulation in an infinite loop.

// gl_window.StartDrawLoop(paramsH.dT);
// return 0;
#endif
}

// =============================================================================

void InitializeMbdPhysicalSystem(ChSystemParallelDVI* mphysicalSystem, int argc, char* argv[]) {
  // Desired number of OpenMP threads (will be clamped to maximum available)
  int threads = 32;
  // Perform dynamic tuning of number of threads?
  bool thread_tuning = true;

  //	uint max_iteration = 20;//10000;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 200;
  int max_iteration_spinning = 0;
  int max_iteration_bilateral = 100;

  //  omp_get_num_procs();
  int max_threads = mphysicalSystem->GetParallelThreadNumber();
  if (threads > max_threads)
    threads = max_threads;
  mphysicalSystem->SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);

  mphysicalSystem->GetSettings()->perform_thread_tuning = thread_tuning;
  mphysicalSystem->GetSettings()->min_threads = max(1, threads / 2);
  mphysicalSystem->GetSettings()->max_threads = int(3.0 * threads / 2);

  // ---------------------
  // Edit mphysicalSystem settings.
  // ---------------------

  double tolerance = 0.1;  // 1e-3;  // Arman, move it to paramsH
  // double collisionEnvelop = 0.04 * paramsH.HSML;
  mphysicalSystem->Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

  mphysicalSystem->GetSettings()->solver.solver_mode = SLIDING;                              // NORMAL, SPINNING
  mphysicalSystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;        // max_iteration / 3
  mphysicalSystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;      // max_iteration / 3
  mphysicalSystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;    // 0
  mphysicalSystem->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;  // max_iteration / 3
  mphysicalSystem->GetSettings()->solver.tolerance = tolerance;
  mphysicalSystem->GetSettings()->solver.alpha = 0;  // Arman, find out what is this
  mphysicalSystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
  mphysicalSystem->ChangeSolverType(APGD);  // Arman check this APGD APGDBLAZE
  //  mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

  //    mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;   // global collisionEnvelop
  //    does not work. Maybe due to sph-tire size mismatch
  mphysicalSystem->GetSettings()->collision.bins_per_axis = _make_int3(40, 40, 40);  // Arman check
}
// =============================================================================
void CreateMbdPhysicalSystemObjects(ChSystemParallelDVI* system) {
  switch (wheel_type) {
    case CYLINDRICAL: {
      mVehicle = new ChWheeledVehicleAssembly(system, vehicle_file_cyl, simplepowertrain_file);
      tire_cb = new MyCylindricalTire();
    } break;
    case LUGGED: {
      mVehicle = new ChWheeledVehicleAssembly(system, vehicle_file_lug, simplepowertrain_file);
      tire_cb = new MyLuggedTire();
    } break;
  }
  driver_cb = new MyDriverInputs(time_hold_vehicle);

  mVehicle->SetTireContactCallback(tire_cb);

  // Set the callback object for driver inputs. Pass the hold time as a delay in
  // generating driver inputs.
  mVehicle->SetDriverInputsCallback(driver_cb);

  // Initialize the mVehicle at a height above the terrain.
  mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

  // Initially, fix the chassis and wheel bodies (will be released after time_hold).
  mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
    mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  }
}
// =============================================================================

int main(int argc, char* argv[]) {
  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);
  ChSystemParallelDVI* system = new ChSystemParallelDVI();
  InitializeMbdPhysicalSystem(system, argc, argv);

  //******************
  //    switch (wheel_type) {
  //        case CYLINDRICAL: {
  //            mVehicle = new ChWheeledVehicleAssembly(system, vehicle_file_cyl, simplepowertrain_file);
  //            tire_cb = new MyCylindricalTire();
  //        } break;
  //        case LUGGED: {
  //            mVehicle = new ChWheeledVehicleAssembly(system, vehicle_file_lug, simplepowertrain_file);
  //            tire_cb = new MyLuggedTire();
  //        } break;
  //    }
  //    driver_cb = new MyDriverInputs(time_hold_vehicle);

  //    mVehicle->SetTireContactCallback(tire_cb);
  //
  //    // Set the callback object for driver inputs. Pass the hold time as a delay in
  //    // generating driver inputs.
  //    MyDriverInputs driver_cb(time_hold_vehicle);
  //    mVehicle->SetDriverInputsCallback(&driver_cb);
  //
  //    // Initialize the mVehicle at a height above the terrain.
  //    mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);
  //
  //    // Initially, fix the chassis and wheel bodies (will be released after time_hold).
  //    mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  //    for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
  //        mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  //    }

  CreateMbdPhysicalSystemObjects(system);
  InitializeChronoGraphics(system);

  // Run simulation for specified time.
  double time = 0;
  while (time < time_end) {
    DoStepChronoSystem(system, time_step, time);  // Keep only this if you are just interested in the rigid sys
    time += time_step;
  }

  delete mVehicle;
  delete tire_cb;
  delete driver_cb;

  return 0;
}
