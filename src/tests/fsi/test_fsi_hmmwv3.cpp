
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

// note: fsi_hmmwv model for leap frog integration. Have not spent much of time on fluid-solid coupling. you need to
// take care of that later

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
#include "chrono_fsi/MyStructs.cuh"  //just for SimParams
#include "chrono_fsi/collideSphereSphere.cuh"
#include "chrono_fsi/printToFile.cuh"
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/checkPointReduced.h"

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

// Chrono Vehicle Include
#include "chrono_fsi/VehicleExtraProperties.h"
//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"
#include "utils/ChUtilsInputOutput.h"

// Chrono general utils
#include "core/ChFileutils.h"

// FSI Interface Includes
#include "chrono_fsi/fsi_hmmwv_params.h"
//#include "BallDropParams.h"
#include "chrono_fsi/SphInterface.h"
#include "chrono_fsi/InitializeSphMarkers.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Define General variables
SimParams paramsH;

#define haveFluid true
#define useWallBce true

#if haveFluid
#else
#undef useWallBce
#define useWallBce false
#endif
// =============================================================================
// Define Graphics
#define irrlichtVisualization false

#if irrlichtVisualization

// Irrlicht Include
#include "unit_IRRLICHT/ChIrrApp.h"

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

std::shared_ptr<ChIrrApp> application;
#endif

//#ifdef CHRONO_OPENGL
//#undef CHRONO_OPENGL
//#endif

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
#endif

// =============================================================================
void SetArgumentsForMbdFromInput(int argc,
                                 char* argv[],
                                 int& threads,
                                 int& max_iteration_sliding,
                                 int& max_iteration_bilateral) {
  if (argc > 1) {
    const char* text = argv[1];
    threads = atoi(text);
  }
  if (argc > 2) {
    const char* text = argv[2];
    max_iteration_sliding = atoi(text);
  }
  if (argc > 3) {
    const char* text = argv[3];
    max_iteration_bilateral = atoi(text);
  }
}
// =============================================================================
void SetArgumentsForMbdFromInput(int argc,
                                 char* argv[],
                                 int& threads,
                                 int& max_iteration_normal,
                                 int& max_iteration_sliding,
                                 int& max_iteration_spinning,
                                 int& max_iteration_bilateral) {
  if (argc > 1) {
    const char* text = argv[1];
    threads = atoi(text);
  }
  if (argc > 2) {
    const char* text = argv[2];
    max_iteration_normal = atoi(text);
  }
  if (argc > 3) {
    const char* text = argv[3];
    max_iteration_sliding = atoi(text);
  }
  if (argc > 4) {
    const char* text = argv[4];
    max_iteration_spinning = atoi(text);
  }
  if (argc > 5) {
    const char* text = argv[5];
    max_iteration_bilateral = atoi(text);
  }
}
// =============================================================================

void InitializeMbdPhysicalSystem(ChSystemParallelDVI& mphysicalSystem, int argc, char* argv[]) {
  // Desired number of OpenMP threads (will be clamped to maximum available)
  int threads = 32;
  // Perform dynamic tuning of number of threads?
  bool thread_tuning = true;

  //	uint max_iteration = 20;//10000;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 200;
  int max_iteration_spinning = 0;
  int max_iteration_bilateral = 100;

  // ----------------------
  // Set params from input
  // ----------------------

  SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration_sliding, max_iteration_bilateral);

  // ----------------------
  // Set number of threads.
  // ----------------------

  int max_threads = omp_get_num_procs();
  if (threads > max_threads)
    threads = max_threads;
  mphysicalSystem.SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);
  cout << "Using " << threads << " threads" << endl;

  mphysicalSystem.GetSettings()->perform_thread_tuning = thread_tuning;
  mphysicalSystem.GetSettings()->min_threads = max(1, threads / 2);
  mphysicalSystem.GetSettings()->max_threads = int(3.0 * threads / 2);

  // ---------------------
  // Print the rest of parameters
  // ---------------------

  simParams << endl << " number of threads: " << threads << endl << " max_iteration_normal: " << max_iteration_normal
            << endl << " max_iteration_sliding: " << max_iteration_sliding << endl
            << " max_iteration_spinning: " << max_iteration_spinning << endl
            << " max_iteration_bilateral: " << max_iteration_bilateral << endl << endl;

  // ---------------------
  // Edit mphysicalSystem settings.
  // ---------------------

  double tolerance = 0.1;  // 1e-3;  // Arman, move it to paramsH
  // double collisionEnvelop = 0.04 * paramsH.HSML;
  mphysicalSystem.Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

  mphysicalSystem.GetSettings()->solver.solver_mode = SLIDING;                              // NORMAL, SPINNING
  mphysicalSystem.GetSettings()->solver.max_iteration_normal = max_iteration_normal;        // max_iteration / 3
  mphysicalSystem.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;      // max_iteration / 3
  mphysicalSystem.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;    // 0
  mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;  // max_iteration / 3
  mphysicalSystem.GetSettings()->solver.use_full_inertia_tensor = true;
  mphysicalSystem.GetSettings()->solver.tolerance = tolerance;
  mphysicalSystem.GetSettings()->solver.alpha = 0;  // Arman, find out what is this
  mphysicalSystem.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
  mphysicalSystem.ChangeSolverType(APGD);  // Arman check this APGD APGDBLAZE
  //  mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

  //    mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;   // global collisionEnvelop
  //    does not work. Maybe due to sph-tire size mismatch
  mphysicalSystem.GetSettings()->collision.bins_per_axis = _make_int3(40, 40, 40);  // Arman check
}
// =============================================================================

double CreateGranularBed(ChSystem* mphysicalSystem) {
  // Create a material

  ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
  mat_g->SetFriction(mu_g);

  // Create a particle generator and a mixture entirely made out of spheres

  utils::Generator gen(mphysicalSystem);
  utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
  m1->setDefaultMaterialDVI(mat_g);
  m1->setDefaultDensity(rho_g);
  m1->setDefaultSize(r_g);

  // Set starting value for body identifiers

  gen.setBodyIdentifier(Id_g);

  // Create particles in layers until reaching the desired number of particles

  double r = 1.01 * r_g;
  ChVector<> hdims(hdimX - r, hdimY - r, 0);
  ChVector<> center(0, 0, 2 * r);

  while (gen.getTotalNumBodies() < num_particles) {
    gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
    center.z += 2 * r;
  }

  return center.z;
}
// =============================================================================
void AddBoxBceToChSystemAndSPH(
    ChBody* body,
    const ChVector<>& size,
    const ChVector<>& pos,
    const ChQuaternion<>& rot,
    bool visualization,

    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    thrust::host_vector< ::int3>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass) {
  utils::AddBoxGeometry(body, size, pos, rot, visualization);

  if (!initializeFluidFromFile) {
#if haveFluid
#if useWallBce
    assert(referenceArray.size() > 1 &&
           "error: fluid need to be initialized before boundary. Reference array should have two components");

    thrust::host_vector<Real3> posRadBCE;
    thrust::host_vector<Real4> velMasBCE;
    thrust::host_vector<Real4> rhoPresMuBCE;

    CreateBCE_On_Box(posRadBCE, velMasBCE, rhoPresMuBCE, paramsH, sphMarkerMass, size, pos, rot, 12);
    int numBCE = posRadBCE.size();
    int numSaved = posRadH.size();
    for (int i = 0; i < numBCE; i++) {
      posRadH.push_back(posRadBCE[i]);
      velMasH.push_back(velMasBCE[i]);
      rhoPresMuH.push_back(rhoPresMuBCE[i]);
      bodyIndex.push_back(i + numSaved);
    }

    ::int3 ref3 = referenceArray[1];
    ref3.y = ref3.y + numBCE;
    referenceArray[1] = ref3;

    int numAllMarkers = numBCE + numSaved;
    SetNumObjects(numObjects, referenceArray, numAllMarkers);

    posRadBCE.clear();
    velMasBCE.clear();
    rhoPresMuBCE.clear();
#endif
#endif
  }
}

// =============================================================================

void CreateMbdPhysicalSystemObjects(
    ChSystemParallelDVI& mphysicalSystem,
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    thrust::host_vector< ::int3>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass) {
  // Ground body
  ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(true);

  ground->GetMaterialSurface()->SetFriction(mu_g);

  ground->GetCollisionModel()->ClearModel();

  // Bottom box
  double hdimSide = hdimX / 4.0;
  double midSecDim = hdimX - 2 * hdimSide;

  // basin info
  double phi = CH_C_PI / 9;
  double bottomWidth = midSecDim - basinDepth / tan(phi);  // for a 45 degree slope
  double bottomBuffer = .4 * bottomWidth;

  double inclinedWidth = 0.5 * basinDepth / sin(phi);  // for a 45 degree slope

  double smallBuffer = .7 * hthick;
  double x1I = -midSecDim + inclinedWidth * cos(phi) - hthick * sin(phi) - smallBuffer;
  double zI = -inclinedWidth * sin(phi) - hthick * cos(phi);
  double x2I = midSecDim - inclinedWidth * cos(phi) + hthick * sin(phi) + smallBuffer;

  // beginning third
  AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                            ChVector<>(hdimSide, hdimY, hthick),
                            ChVector<>(-midSecDim - hdimSide, 0, -hthick),
                            ChQuaternion<>(1, 0, 0, 0),
                            true,
                            posRadH,
                            velMasH,
                            rhoPresMuH,
                            bodyIndex,
                            referenceArray,
                            numObjects,
                            paramsH,
                            sphMarkerMass);

  // end third
  AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                            ChVector<>(hdimSide, hdimY, hthick),
                            ChVector<>(midSecDim + hdimSide, 0, -hthick),
                            ChQuaternion<>(1, 0, 0, 0),
                            true,
                            posRadH,
                            velMasH,
                            rhoPresMuH,
                            bodyIndex,
                            referenceArray,
                            numObjects,
                            paramsH,
                            sphMarkerMass);
  // basin
  AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                            ChVector<>(bottomWidth + bottomBuffer, hdimY, hthick),
                            ChVector<>(0, 0, -basinDepth - hthick),
                            ChQuaternion<>(1, 0, 0, 0),
                            true,
                            posRadH,
                            velMasH,
                            rhoPresMuH,
                            bodyIndex,
                            referenceArray,
                            numObjects,
                            paramsH,
                            sphMarkerMass);
  // slope 1
  AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                            ChVector<>(inclinedWidth, hdimY, hthick),
                            ChVector<>(x1I, 0, zI),
                            Q_from_AngAxis(phi, ChVector<>(0, 1, 0)),
                            true,
                            posRadH,
                            velMasH,
                            rhoPresMuH,
                            bodyIndex,
                            referenceArray,
                            numObjects,
                            paramsH,
                            sphMarkerMass);

  // slope 2
  AddBoxBceToChSystemAndSPH(ground.get_ptr(),
                            ChVector<>(inclinedWidth, hdimY, hthick),
                            ChVector<>(x2I, 0, zI),
                            Q_from_AngAxis(-phi, ChVector<>(0, 1, 0)),
                            true,
                            posRadH,
                            velMasH,
                            rhoPresMuH,
                            bodyIndex,
                            referenceArray,
                            numObjects,
                            paramsH,
                            sphMarkerMass);

  // a flat surface altogether
  //  utils::AddBoxGeometry(
  //      ground.get_ptr(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0),
  //      true);

  if (initializeFluidFromFile) {
    if (numObjects.numBoundaryMarkers > 0) {
      ground->GetCollisionModel()->SetFamily(fluidCollisionFamily);
      ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(fluidCollisionFamily);
    }
  } else {
#if haveFluid
#if useWallBce
    ground->GetCollisionModel()->SetFamily(fluidCollisionFamily);
    ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(fluidCollisionFamily);
#endif
#endif
  }

  ground->GetCollisionModel()->BuildModel();

  mphysicalSystem.AddBody(ground);

  // version 1
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
  // Set the callback object for chassis.
  switch (chassis_type) {
    case CBOX: {
      chassis_cb = new MyChassisBoxModel_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
      ChVector<> boxSize(1, .5, .2);
      ((MyChassisBoxModel_vis*)chassis_cb)->SetAttributes(boxSize);
      mVehicle->SetChassisContactCallback(chassis_cb);
    } break;

    case CSPHERE: {
      chassis_cb = new MyChassisSphereModel_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
      Real radius = 1;
      ((MyChassisSphereModel_vis*)chassis_cb)->SetAttributes(radius);
      mVehicle->SetChassisContactCallback(chassis_cb);
    } break;

    case C_SIMPLE_CONVEX_MESH: {
      chassis_cb = new MyChassisSimpleConvexMesh();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
      mVehicle->SetChassisContactCallback(chassis_cb);
    } break;

    case C_SIMPLE_TRI_MESH: {
      chassis_cb = new MyChassisSimpleTriMesh_vis();  //(mVehicle->GetVehicle()->GetChassis(), ChVector<>(1, .5, .4));
      mVehicle->SetChassisContactCallback(chassis_cb);
    } break;
  }

  // Set the callback object for driver inputs. Pass the hold time as a delay in
  // generating driver inputs.
  driver_cb = new MyDriverInputs(time_hold_vehicle);
  mVehicle->SetDriverInputsCallback(driver_cb);

  // Initialize the vehicle at a height above the terrain.
  mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

  // Initially, fix the chassis (will be released after time_hold_vehicle).
  mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  // Initially, fix the wheels (will be released after time_hold_vehicle).
  for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
    mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  }

  //  // version 2
  //  // -----------------------------------------
  //  // Create and initialize the vehicle system.
  //  // -----------------------------------------
  //  std::string vehicle_file_cyl1("hmmwv/vehicle/HMMWV_Vehicle_simple.json");
  //  std::string vehicle_file_lug1("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");
  //
  //  // JSON files for powertrain (simple)
  //  std::string simplepowertrain_file1("hmmwv/powertrain/HMMWV_SimplePowertrain.json");
  //
  //  // Create the vehicle assembly and the callback object for tire contact
  //  // according to the specified type of tire/wheel.
  //  switch (wheel_type) {
  //      case CYLINDRICAL: {
  //    	  mVehicle = new ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_cyl1, simplepowertrain_file1);
  //          tire_cb = new MyCylindricalTire();
  //      } break;
  //      case LUGGED: {
  //    	  mVehicle = new ChWheeledVehicleAssembly(&mphysicalSystem, vehicle_file_lug1, simplepowertrain_file1);
  //          tire_cb = new MyLuggedTire();
  //      } break;
  //  }
  //
  //  mVehicle->SetTireContactCallback(tire_cb);
  //
  //  // Set the callback object for driver inputs. Pass the hold time as a delay in
  //  // generating driver inputs.
  //  driver_cb = new MyDriverInputs(time_hold_vehicle);
  //  mVehicle->SetDriverInputsCallback(driver_cb);
  //
  //  // Initialize the vehicle at a height above the terrain.
  //  mVehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);
  //
  //  // Initially, fix the chassis and wheel bodies (will be released after time_hold).
  //  mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  //  for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
  //	  mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  //  }

  // extra objects
  // -----------------------------------------
  // Add extra collision body to test the collision shape
  // -----------------------------------------
  //
  //  Real rad = 0.1;
  //  // NOTE: mass properties and shapes are all for sphere
  //  double volume = utils::CalcSphereVolume(rad);
  //  ChVector<> gyration = utils::CalcSphereGyration(rad).Get_Diag();
  //  double density = paramsH.rho0;
  //  double mass = density * volume;
  //  double muFriction = 0;
  //
  //  // Create a common material
  //  ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
  //  mat_g->SetFriction(muFriction);
  //  mat_g->SetCohesion(0);
  //  mat_g->SetCompliance(0.0);
  //  mat_g->SetComplianceT(0.0);
  //  mat_g->SetDampingF(0.2);
  //
  //  for (Real x = -4; x < 2; x += 0.25) {
  //    for (Real y = -1; y < 1; y += 0.25) {
  //      ChSharedPtr<ChBody> mball = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
  //      ChVector<> pos = ChVector<>(-8.5, .20, 3) + ChVector<>(x, y, 0);
  //      mball->SetMaterialSurface(mat_g);
  //      // body->SetIdentifier(fId);
  //      mball->SetPos(pos);
  //      mball->SetCollide(true);
  //      mball->SetBodyFixed(false);
  //      mball->SetMass(mass);
  //      mball->SetInertiaXX(mass * gyration);
  //
  //      mball->GetCollisionModel()->ClearModel();
  //      utils::AddSphereGeometry(mball.get_ptr(), rad);  // O
  //                                                       //	utils::AddEllipsoidGeometry(body.get_ptr(), size);
  //                                                       // X
  //
  //      mball->GetCollisionModel()->SetFamily(100);
  //      mball->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(100);
  //
  //      mball->GetCollisionModel()->BuildModel();
  //      mphysicalSystem.AddBody(mball);
  //    }
  //  }
}
// =============================================================================

void InitializeChronoGraphics(ChSystemParallelDVI& mphysicalSystem) {
  //	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
  //	ChVector<> CameraLocation = ChVector<>(2 * paramsH.cMax.x, 2 * paramsH.cMax.y, 2 * paramsH.cMax.z);
  //	ChVector<> CameraLookAt = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);
  ChVector<> CameraLocation = ChVector<>(0, -10, 0);
  ChVector<> CameraLookAt = ChVector<>(0, 0, 0);

#ifdef CHRONO_OPENGL
  gl_window.Initialize(1280, 720, "HMMWV", &mphysicalSystem);
  gl_window.SetCamera(CameraLocation, CameraLookAt, ChVector<>(0, 0, 1));
  gl_window.SetRenderMode(opengl::WIREFRAME);

// Uncomment the following two lines for the OpenGL manager to automatically un the simulation in an infinite loop.

// gl_window.StartDrawLoop(time_step);
// return 0;
#endif

#if irrlichtVisualization
  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  application = std::shared_ptr<ChIrrApp>(
      new ChIrrApp(&mphysicalSystem, L"Bricks test", core::dimension2d<u32>(800, 600), false, true));
  //	ChIrrApp application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  ChIrrWizard::add_typical_Logo(application->GetDevice());
  //		ChIrrWizard::add_typical_Sky   (application->GetDevice());
  ChIrrWizard::add_typical_Lights(
      application->GetDevice(), core::vector3df(14.0f, 44.0f, -18.0f), core::vector3df(-3.0f, 8.0f, 6.0f), 59, 40);
  ChIrrWizard::add_typical_Camera(
      application->GetDevice(),
      core::vector3df(CameraLocation.x, CameraLocation.y, CameraLocation.z),
      core::vector3df(CameraLookAt.x, CameraLookAt.y, CameraLookAt.z));  //   (7.2,30,0) :  (-3,12,-8)
  // Use this function for adding a ChIrrNodeAsset to all items
  // If you need a finer control on which item really needs a visualization proxy in
  // Irrlicht, just use application->AssetBind(myitem); on a per-item basis.
  application->AssetBindAll();
  // Use this function for 'converting' into Irrlicht meshes the assets
  // into Irrlicht-visualizable meshes
  application->AssetUpdateAll();

  application->SetStepManage(true);
#endif
}
// =============================================================================

void SavePovFilesMBD(ChSystemParallelDVI& mphysicalSystem,
                     int tStep,
                     double mTime,
                     int& num_contacts,
                     double exec_time) {
  int out_steps = std::ceil((1.0 / time_step) / out_fps);

  static int out_frame = 0;

  // If enabled, output data for PovRay postprocessing.
  if (povray_output && tStep % out_steps == 0) {
    if (tStep / out_steps == 0) {
      const std::string rmCmd = std::string("rm ") + pov_dir_mbd + std::string("/*.dat");
      system(rmCmd.c_str());
    }
    char filename[100];
    sprintf(filename, "%s/data_%03d.dat", pov_dir_mbd.c_str(), out_frame + 1);
    utils::WriteShapesPovray(&mphysicalSystem, filename);

    cout << "------------ Output frame:   " << out_frame + 1 << endl;
    cout << "             Sim frame:      " << tStep << endl;
    cout << "             Time:           " << mTime << endl;
    cout << "             Avg. contacts:  " << num_contacts / out_steps << endl;
    cout << "             Execution time: " << exec_time << endl;

    out_frame++;
    num_contacts = 0;
  }
}
// =============================================================================
void SetMarkersVelToZero(thrust::device_vector<Real4>& velMasD, thrust::host_vector<Real4>& velMasH) {
  for (int i = 0; i < velMasH.size(); i++) {
    Real4 vM = velMasH[i];
    velMasH[i] = mR4(0, 0, 0, vM.w);
    velMasD[i] = mR4(0, 0, 0, vM.w);
  }
}
// =============================================================================
void printSimulationParameters() {
  simParams << " time_hold_vehicle: " << time_hold_vehicle << endl
            << " time_pause_fluid_external_force: " << time_pause_fluid_external_force << endl
            << " contact_recovery_speed: " << contact_recovery_speed << endl << " maxFlowVelocity " << maxFlowVelocity
            << endl << " time_step: " << time_step << endl << " time_end: " << time_end << endl;
}
// =============================================================================

int DoStepChronoSystem(ChSystemParallelDVI& mphysicalSystem, Real dT, double mTime) {
  // Release the vehicle chassis at the end of the hold time.
  if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed() && mTime > time_hold_vehicle) {
    mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
    for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles(); i++) {
      mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
    }
  }

  // Update vehicle
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
  mphysicalSystem.DoStepDynamics(dT);
#endif
#endif
  return 1;
}
// =============================================================================

int main(int argc, char* argv[]) {
  //****************************************************************************************
  time_t rawtime;
  struct tm* timeinfo;

  GpuTimer myGpuTimerHalfStep;
  ChTimer<double> myCpuTimerHalfStep;
  //(void) cudaSetDevice(0);

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  //****************************************************************************************
  SetChronoDataPath(CHRONO_DATA_DIR);

  // --------------------------
  // Create output directories.
  // --------------------------

  if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    cout << "Error creating directory " << out_dir << endl;
    return 1;
  }

  if (povray_output) {
    if (ChFileutils::MakeDirectory(pov_dir_mbd.c_str()) < 0) {
      cout << "Error creating directory " << pov_dir_mbd << endl;
      return 1;
    }
  }

  if (ChFileutils::MakeDirectory(pov_dir_fluid.c_str()) < 0) {
    cout << "Error creating directory " << pov_dir_fluid << endl;
    return 1;
  }

  //****************************************************************************************
  const std::string simulationParams = out_dir + "/simulation_specific_parameters.txt";
  simParams.open(simulationParams);
  simParams << " Job was submittet at date/time: " << asctime(timeinfo) << endl;
  printSimulationParameters();

  // ***************************** Create Fluid ********************************************
  thrust::host_vector< ::int3> referenceArray;
  thrust::host_vector<Real3> posRadH;  // do not set the size here since you are using push back later
  thrust::host_vector<Real4> velMasH;
  thrust::host_vector<Real4> rhoPresMuH;
  thrust::host_vector<uint> bodyIndex;
  Real sphMarkerMass = 0;  // To be initialized in CreateFluidMarkers, and used in other places

  SetupParamsH(paramsH);

  if (initializeFluidFromFile) {
    CheckPointMarkers_Read(
        initializeFluidFromFile, posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray, paramsH, numObjects);
    if (numObjects.numAllMarkers == 0) {
      ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
      return 0;
    }
#if haveFluid
#else
    printf("Error! Initialized from file But haveFluid is false! \n");
    return;
#endif
  } else {
#if haveFluid

    //*** default num markers

    int numAllMarkers = 0;

    //*** initialize fluid particles
    ::int2 num_fluidOrBoundaryMarkers =
        CreateFluidMarkers(posRadH, velMasH, rhoPresMuH, bodyIndex, paramsH, sphMarkerMass);
    printf("num_fluidOrBoundaryMarkers %d %d \n", num_fluidOrBoundaryMarkers.x, num_fluidOrBoundaryMarkers.y);
    referenceArray.push_back(mI3(0, num_fluidOrBoundaryMarkers.x, -1));  // map fluid -1
    numAllMarkers += num_fluidOrBoundaryMarkers.x;
    referenceArray.push_back(mI3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y, 0));
    numAllMarkers += num_fluidOrBoundaryMarkers.y;

    //*** set num objects

    SetNumObjects(numObjects, referenceArray, numAllMarkers);
    assert(posRadH.size() == numObjects.numAllMarkers && "numObjects is not set correctly");
    if (numObjects.numAllMarkers == 0) {
      ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
      return 0;
    }
#endif
  }
  // ***************************** Create Rigid ********************************************
  ChSystemParallelDVI mphysicalSystem;
  InitializeMbdPhysicalSystem(mphysicalSystem, argc, argv);

  // This needs to be called after fluid initialization because I am using "numObjects.numBoundaryMarkers" inside it
  CreateMbdPhysicalSystemObjects(
      mphysicalSystem, posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray, numObjects, paramsH, sphMarkerMass);

  // ***************************** Create Interface ********************************************

  //*** Add sph data to the physics system

  int startIndexSph = 0;
#if haveFluid
  AddSphDataToChSystem(
      mphysicalSystem, startIndexSph, posRadH, velMasH, paramsH, numObjects, fluidCollisionFamily, sphMarkerMass);

  thrust::device_vector<Real3> posRadD = posRadH;
  thrust::device_vector<Real4> velMasD = velMasH;
  thrust::device_vector<Real4> rhoPresMuD = rhoPresMuH;
  thrust::device_vector<uint> bodyIndexD = bodyIndex;
  thrust::device_vector<Real4> derivVelRhoD;

  thrust::device_vector<Real4> rhoPresMuD_half = rhoPresMuD;
  thrust::device_vector<Real4> velMasD_half = velMasD;

  ResizeMyThrust4(derivVelRhoD, numObjects.numAllMarkers);
#endif
  cout << " -- ChSystem size : " << mphysicalSystem.Get_bodylist()->size() << endl;

  // ***************************** System Initialize ********************************************

  InitializeChronoGraphics(mphysicalSystem);

  double mTime = 0;
  double exec_time = 0;
  int num_contacts = 0;

  DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");

  int stepEnd = int(paramsH.tFinal / paramsH.dT);  // 1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) /
                                                   // paramsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) /
                                                   // paramsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) /
                                                   // paramsH.dT ;//0.7e6;//2.5e6;
                                                   // //200000;//10000;//50000;//100000;
  printf("stepEnd %d\n", stepEnd);
  Real realTime = 0;

  printf("\ntimePause %f, numPause %d\n", paramsH.timePause, int(paramsH.timePause / paramsH.dT));
  printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n",
         paramsH.timePauseRigidFlex,
         int((paramsH.timePauseRigidFlex - paramsH.timePause) / paramsH.dT + paramsH.timePause / paramsH.dT));

  InitSystem(paramsH, numObjects);
  mphysicalSystem.Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

  simParams.close();
// ***************************** Middle values for leap-frog (LF) ********************************************
#if haveFluid
  ForceSPH_LF(posRadD,
              velMasD,
              rhoPresMuD,
              bodyIndexD,
              derivVelRhoD,
              referenceArray,
              numObjects,
              paramsH,
              bceType,
              0.5 * paramsH.dT);

  UpdateFluid_init_LF(posRadD, velMasD_half, rhoPresMuD_half, derivVelRhoD, referenceArray, paramsH.dT);

//  printf("******************* v1 %f %f %f and v2 %f %f %f and dd %f %f %f %f and p1 %f %f %f and p2 %f %f %f\n", v1.x,
//  v1.y, v1.z, v2.x, v2.y, v2.z, dd.x, dd.y, dd.z, dd.w, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);

#endif
  // ***************************** Simulation loop ********************************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  // *****************************
  for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
    // -------------------
    // SPH Block
    // -------------------
    myCpuTimerHalfStep.start();
    myGpuTimerHalfStep.Start();
    chrono::ChTimerParallel fsi_timer;
    fsi_timer.Reset();

#if haveFluid
    CpuTimer mCpuTimer;
    mCpuTimer.Start();
    GpuTimer myGpuTimer;
    myGpuTimer.Start();

//		CopySys2D(posRadD, mphysicalSystem, numObjects, startIndexSph);

#endif
    // -------------------
    // End SPH Block
    // -------------------

    // If enabled, output data for PovRay postprocessing.
    SavePovFilesMBD(mphysicalSystem, tStep, mTime, num_contacts, exec_time);

// ****************** RK2: 1/2
#if haveFluid

    UpdateFluid_rho_vel_LF(
        velMasD, rhoPresMuD, velMasD_half, rhoPresMuD_half, derivVelRhoD, referenceArray, 0.5 * paramsH.dT);

    // At this point, posRadD, velMasD, and rhoPresMuD are all the same time
    ApplyBoundarySPH_Markers(posRadD, rhoPresMuD, numObjects.numAllMarkers);

    FillMyThrust4(derivVelRhoD, mR4(0));

    fsi_timer.start("fluid_initialization");
    int out_steps = std::ceil((1.0 / time_step) / out_fps);
    PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, paramsH, realTime, tStep, out_steps, pov_dir_fluid);

    // ******* slow down the sys.Check point the sys.

    CheckPointMarkers_Write(
        posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray, paramsH, numObjects, tStep, tStepsCheckPoint);

    //    if (fmod(realTime, 0.6) < time_step && realTime < 1.3) {
    //    	SetMarkersVelToZero(velMasD, velMasH);
    //    }
    // *******
    fsi_timer.stop("fluid_initialization");

    fsi_timer.start("force_sph");

    ForceSPH_LF(posRadD,
                velMasD,
                rhoPresMuD,
                bodyIndexD,
                derivVelRhoD,
                referenceArray,
                numObjects,
                paramsH,
                bceType,
                paramsH.dT);  // Arman later take care of dT. It is useless and you can remove it.

    fsi_timer.stop("force_sph");
    UpdateFluid_EveryThing_LF(posRadD, velMasD_half, rhoPresMuD_half, derivVelRhoD, referenceArray, paramsH.dT);

    if ((tStep % 10 == 0) && (paramsH.densityReinit != 0)) {
      DensityReinitialization(posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, paramsH.gridSize);
    }

#endif

    fsi_timer.start("stepDynamic_mbd");
    DoStepChronoSystem(
        mphysicalSystem, paramsH.dT, mTime);  // Keep only this if you are just interested in the rigid sys
    fsi_timer.stop("stepDynamic_mbd");

#if haveFluid
    CopyD2H(posRadH, velMasH, rhoPresMuH, posRadD, velMasD, rhoPresMuD);
    UpdateSphDataInChSystem(mphysicalSystem, posRadH, velMasH, numObjects, startIndexSph);
#endif

    // Update counters.
    mTime += time_step;
    exec_time += mphysicalSystem.GetTimerStep();
    num_contacts += mphysicalSystem.GetNcontacts();

// -------------------
// SPH Block
// -------------------

#if haveFluid

    mCpuTimer.Stop();
    myGpuTimer.Stop();
    if (tStep % 2 == 0) {
      printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ",
             tStep,
             realTime,
             (Real)myGpuTimer.Elapsed(),
             1000 * mCpuTimer.Elapsed());

      fsi_timer.stop("half_step_dynamic_fsi_22");
      fsi_timer.PrintReport();
    }
#endif
    // -------------------
    // End SPH Block
    // -------------------

    fflush(stdout);
    realTime += paramsH.dT;
    mphysicalSystem.data_manager->system_timer.PrintReport();
  }
  ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
#if haveFluid
  ClearMyThrustR3(posRadD);
  ClearMyThrustR4(velMasD);
  ClearMyThrustR4(rhoPresMuD);
  ClearMyThrustU1(bodyIndexD);
  ClearMyThrustR4(derivVelRhoD);
  ClearMyThrustR4(velMasD_half);
  ClearMyThrustR4(rhoPresMuD_half);
#endif
  delete mVehicle;
  delete tire_cb;
  delete chassis_cb;
  delete driver_cb;

  return 0;
}
