
///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description
//					reads the number of particles first. The each line provides the
//					properties of one SPH particl:
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu,
// particle_type(rigid
// or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

// note: this is the original fsi_hmmwv model. uses RK2, an specific coupling, and density re_initializaiton.

// General Includes
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <ctime>
#include <assert.h>
#include <stdlib.h>  // system

// SPH includes
#include "MyStructs.cuh"  //just for SimParams
#include "collideSphereSphere.cuh"
#include "printToFile.cuh"
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "checkPointReduced.h"

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

// Chrono Vehicle Include
#include "VehicleExtraProperties.h"
#include "chrono_vehicle/ChVehicleModelData.h"

//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"
#include "utils/ChUtilsInputOutput.h"

// Chrono general utils
#include "core/ChFileutils.h"
#include <core/ChTransform.h> //transform acc from GF to LF for post process

// FSI Interface Includes
#include "fsi_hmmwv_params.h"
//#include "BallDropParams.h"
#include "SphInterface.h"
#include "InitializeSphMarkers.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Define General variables
SimParams paramsH;

#define haveFluid false
#define useWallBce true

#if haveFluid
#else
#undef useWallBce
#define useWallBce false
#endif
// =============================================================================
// Define Graphics
//#ifdef CHRONO_OPENGL
//#undef CHRONO_OPENGL
//#endif

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
#endif

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
  mphysicalSystem->GetSettings()->min_threads = max(1, threads/2);
  mphysicalSystem->GetSettings()->max_threads = int(3.0 * threads / 2);

  // ---------------------
  // Edit mphysicalSystem settings.
  // ---------------------

  double tolerance = 0.1;  // 1e-3;  // Arman, move it to paramsH
  //double collisionEnvelop = 0.04 * paramsH.HSML;
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

//    mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;   // global collisionEnvelop does not work. Maybe due to sph-tire size mismatch
  mphysicalSystem->GetSettings()->collision.bins_per_axis = _make_int3(40, 40, 40);  // Arman check
}

// =============================================================================

void CreateMbdPhysicalSystemObjects(ChSystemParallelDVI& mphysicalSystem) {
  // -----------------------------------------
  // Create and initialize the vehicle system.
  // -----------------------------------------
  // Create the vehicle assembly and the callback object for tire contact
  // according to the specified type of tire/wheel.
  switch (wheel_type) {
      case CYLINDRICAL: {
    	  mVehicle = new ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_cyl, simplepowertrain_file);
          tire_cb = new MyCylindricalTire();
      } break;
      case LUGGED: {
    	  mVehicle = new ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_lug, simplepowertrain_file);
          tire_cb = new MyLuggedTire();
      } break;
  }

  mVehicle->SetTireContactCallback(tire_cb);

  // Set the callback object for driver inputs. Pass the hold time as a delay in
  // generating driver inputs.
  MyDriverInputs driver_cb(time_hold_vehicle);
  mVehicle->SetDriverInputsCallback(&driver_cb);

  // Initialize the vehicle at a height above the terrain.
  mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

  // Initially, fix the chassis and wheel bodies (will be released after time_hold).
  mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
	  mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  }

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
int DoStepChronoSystem(ChSystemParallelDVI* mphysicalSystem, double dT, double mTime) {

	// Release the mVehicle chassis at the end of the hold time.
  if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed() && mTime > time_hold_vehicle) {
    mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
    for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
      mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
    }
  }

  // Update mVehicle
  mVehicle->Update(mTime);

  printf(" b3 ********* \n");

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
	    printf(" b4 ********* \n");
    gl_window.DoStepDynamics(dT);
    printf(" b5 ********* \n");
    gl_window.Render();
    printf(" b6 ********* \n");
  }
#else
  mphysicalSystem->DoStepDynamics(dT);
#endif
#endif
  return 1;
}
// =============================================================================

int main(int argc, char* argv[]) {

    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    ChSystemParallelDVI* system = new ChSystemParallelDVI();
    InitializeMbdPhysicalSystem(system, argc, argv);

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
    mVehicle->SetTireContactCallback(tire_cb);

    // Set the callback object for driver inputs. Pass the hold time as a delay in
    // generating driver inputs.
    MyDriverInputs driver_cb(time_hold_vehicle);
    mVehicle->SetDriverInputsCallback(&driver_cb);

    // Initialize the mVehicle at a height above the terrain.
    mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

    // Initially, fix the chassis and wheel bodies (will be released after time_hold).
    mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
    for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
        mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
    }


//    CreateMbdPhysicalSystemObjects(system, mVehicle, tire_cb);
    InitializeChronoGraphics(system);
    double mTime = 0;
  // ***************************** Simulation loop ********************************************

    while (mTime < time_end) {
    	DoStepChronoSystem(
    		system, time_step, mTime);  // Keep only this if you are just interested in the rigid sys
    mTime += time_step;
  }
  delete mVehicle;
  delete tire_cb;
  delete chassis_cb;
//  delete driver_cb;

  return 0;
}
