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
// Chrono::Vehicle + ChronoParallel demo program for simulating a HMMWV vehicle
// over rigid or granular material.
//
// The vehicle model uses the utility class ChWheeledVehicleAssembly and is
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

// =============================================================================

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID, GRANULAR };

// Type of terrain
TerrainType terrain_type = RIGID;

// Control visibility of containing bin walls
bool visible_walls = false;

// Dimensions
double hdimX = 5.5;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.25;

// Parameters for granular material
int Id_g = 100;
double r_g = 0.02;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);

float mu_g = 0.8;

int num_particles = 100;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

enum WheelType { CYLINDRICAL, LUGGED };

// Type of wheel/tire (controls both contact and visualization)
WheelType wheel_type = CYLINDRICAL;

// JSON files for vehicle model (using different wheel visualization meshes)
std::string vehicle_file_cyl("hmmwv/vehicle/HMMWV_Vehicle_simple.json");
std::string vehicle_file_lug("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");

// JSON files for powertrain (simple)
std::string simplepowertrain_file("hmmwv/powertrain/HMMWV_SimplePowertrain.json");

// Initial vehicle position and orientation
ChVector<> initLoc(-hdimX + 2.5, 0, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Coefficient of friction
float mu_t = 0.8;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
double time_step = 2e-4;

double tolerance = 0.1;

int max_iteration_bilateral = 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output
bool povray_output = false;

const std::string out_dir = "../HMMWV";
const std::string pov_dir = out_dir + "/POVRAY";

int out_fps = 60;

// =============================================================================

// Callback class for providing driver inputs.
class MyDriverInputs : public ChDriverInputsCallback {
 public:
  MyDriverInputs(double delay) : m_delay(delay) {}

  virtual void onCallback(double time, double& throttle, double& steering, double& braking) {
    throttle = 0;
    steering = 0;
    braking = 0;

    double eff_time = time - m_delay;

    // Do not generate any driver inputs for a duration equal to m_delay.
    if (eff_time < 0)
      return;

    if (eff_time > 0.2)
      throttle = 1.0;
    else if (eff_time > 0.1)
      throttle = 10 * (eff_time - 0.1);
  }

 private:
  double m_delay;
};

// Callback class for specifying rigid tire contact model.
// This version uses cylindrical contact shapes.
class MyCylindricalTire : public ChTireContactCallback {
 public:
  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
    wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

    wheelBody->GetCollisionModel()->ClearModel();
    wheelBody->GetCollisionModel()->AddCylinder(0.46, 0.46, width / 2);
    wheelBody->GetCollisionModel()->BuildModel();

    wheelBody->GetMaterialSurface()->SetFriction(mu_t);
  }
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
class MyLuggedTire : public ChTireContactCallback {
 public:
  MyLuggedTire() {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    geometry::ChTriangleMeshConnected lugged_mesh;
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    num_hulls = lugged_convex.GetHullCount();
  }

  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
    wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

    ChCollisionModelParallel* coll_model = (ChCollisionModelParallel*)wheelBody->GetCollisionModel();
    coll_model->ClearModel();

    // Assemble the tire contact from 15 segments, properly offset.
    // Each segment is further decomposed in convex hulls.
    for (int iseg = 0; iseg < 15; iseg++) {
      ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
      for (int ihull = 0; ihull < num_hulls; ihull++) {
        std::vector<ChVector<> > convexhull;
        lugged_convex.GetConvexHullResult(ihull, convexhull);
        coll_model->AddConvexHull(convexhull, VNULL, rot);
      }
    }

    // Add a cylinder to represent the wheel hub.
    coll_model->AddCylinder(0.223, 0.223, 0.126);

    coll_model->BuildModel();

    wheelBody->GetMaterialSurface()->SetFriction(mu_t);
  }

 private:
  ChConvexDecompositionHACDv2 lugged_convex;
  int num_hulls;
};

// Callback class for specifying rigid tire contact model.
// This version uses a collection of convex contact shapes (meshes).
// In addition, this version overrides the visualization assets of the provided
// wheel body with the collision meshes.
class MyLuggedTire_vis : public ChTireContactCallback {
 public:
  MyLuggedTire_vis() {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
  }

  virtual void onCallback(ChSharedPtr<ChBody> wheelBody, double radius, double width) {
    wheelBody->ChangeCollisionModel(new collision::ChCollisionModelParallel);

    // Clear any existing assets (will be overriden)
    wheelBody->GetAssets().clear();

    wheelBody->GetCollisionModel()->ClearModel();
    for (int j = 0; j < 15; j++) {
      utils::AddConvexCollisionModel(
          wheelBody, lugged_mesh, lugged_convex, VNULL, Q_from_AngAxis(j * 24 * CH_C_DEG_TO_RAD, VECT_Y), false);
    }
    // This cylinder acts like the rims
    utils::AddCylinderGeometry(wheelBody.get_ptr(), 0.223, 0.126);
    wheelBody->GetCollisionModel()->BuildModel();

    wheelBody->GetMaterialSurface()->SetFriction(mu_t);
  }

 private:
  ChConvexDecompositionHACDv2 lugged_convex;
  geometry::ChTriangleMeshConnected lugged_mesh;
};

// =============================================================================

double CreateParticles(ChSystem* system) {
  // Create a material
  ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
  mat_g->SetFriction(mu_g);

  // Create a particle generator and a mixture entirely made out of spheres
  utils::Generator gen(system);
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

  cout << "Created " << gen.getTotalNumBodies() << " particles." << endl;

  return center.z;
}

// =============================================================================

int main(int argc, char* argv[]) {
  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);

  // --------------------------
  // Create output directories.
  // --------------------------

  if (povray_output) {
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
      cout << "Error creating directory " << out_dir << endl;
      return 1;
    }
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
      cout << "Error creating directory " << pov_dir << endl;
      return 1;
    }
  }

  // --------------
  // Create system.
  // --------------

  ChSystemParallelDVI* system = new ChSystemParallelDVI();

  system->Set_G_acc(ChVector<>(0, 0, -9.81));

  // ----------------------
  // Enable debug log
  // ----------------------

  ////system->SetLoggingLevel(LOG_INFO, true);
  ////system->SetLoggingLevel(LOG_TRACE, true);

  // ----------------------
  // Set number of threads.
  // ----------------------

  int max_threads = omp_get_num_procs();
  if (threads > max_threads)
    threads = max_threads;
  system->SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);
  cout << "Using " << threads << " threads" << endl;

  system->GetSettings()->perform_thread_tuning = thread_tuning;

  // ---------------------
  // Edit system settings.
  // ---------------------

  system->GetSettings()->solver.use_full_inertia_tensor = false;

  system->GetSettings()->solver.tolerance = tolerance;

  system->GetSettings()->solver.solver_mode = SLIDING;
  system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
  system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
  system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
  system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
  system->GetSettings()->solver.alpha = 0;
  system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
  system->ChangeSolverType(APGD);

  system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
  system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
  system->GetSettings()->collision.bins_per_axis = I3(10, 10, 10);

  // -------------------
  // Create the terrain.
  // -------------------

  // Ground body
  ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(true);

  ground->GetMaterialSurface()->SetFriction(mu_g);

  ground->GetCollisionModel()->ClearModel();

  // Bottom box
  utils::AddBoxGeometry(
      ground.get_ptr(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick), ChQuaternion<>(1, 0, 0, 0), true);
  if (terrain_type == GRANULAR) {
    // Front box
    utils::AddBoxGeometry(ground.get_ptr(),
                          ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick),
                          ChQuaternion<>(1, 0, 0, 0),
                          visible_walls);
    // Rear box
    utils::AddBoxGeometry(ground.get_ptr(),
                          ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick),
                          ChQuaternion<>(1, 0, 0, 0),
                          visible_walls);
    // Left box
    utils::AddBoxGeometry(ground.get_ptr(),
                          ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick),
                          ChQuaternion<>(1, 0, 0, 0),
                          visible_walls);
    // Right box
    utils::AddBoxGeometry(ground.get_ptr(),
                          ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick),
                          ChQuaternion<>(1, 0, 0, 0),
                          visible_walls);
  }

  ground->GetCollisionModel()->BuildModel();

  system->AddBody(ground);

  // Create the granular material.
  double vertical_offset = 0;

  if (terrain_type == GRANULAR) {
    vertical_offset = CreateParticles(system);
  }

  // -----------------------------------------
  // Create and initialize the vehicle system.
  // -----------------------------------------

  ChWheeledVehicleAssembly* vehicle;
  ChTireContactCallback* tire_cb;

  // Create the vehicle assembly and the callback object for tire contact
  // according to the specified type of tire/wheel.
  switch (wheel_type) {
    case CYLINDRICAL: {
      vehicle = new ChWheeledVehicleAssembly(system, vehicle_file_cyl, simplepowertrain_file);
      tire_cb = new MyCylindricalTire();
    } break;
    case LUGGED: {
      vehicle = new ChWheeledVehicleAssembly(system, vehicle_file_lug, simplepowertrain_file);
      tire_cb = new MyLuggedTire();
    } break;
  }

  vehicle->SetTireContactCallback(tire_cb);

  // Set the callback object for driver inputs. Pass the hold time as a delay in
  // generating driver inputs.
  MyDriverInputs driver_cb(time_hold);
  vehicle->SetDriverInputsCallback(&driver_cb);

  // Initialize the vehicle at a height above the terrain.
  vehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

  // Initially, fix the chassis and wheel bodies (will be released after time_hold).
  vehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);
  for (int i = 0; i < 2 * vehicle->GetVehicle()->GetNumberAxles(); i++) {
    vehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(true);
  }

// -----------------------
// Perform the simulation.
// -----------------------

#ifdef CHRONO_OPENGL
  // Initialize OpenGL
  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
  gl_window.Initialize(1280, 720, "HMMWV (DVI contact)", system);
  gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
  gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

  // Run simulation for specified time.
  int out_steps = std::ceil((1.0 / time_step) / out_fps);

  double time = 0;
  int sim_frame = 0;
  int out_frame = 0;
  int next_out_frame = 0;
  double exec_time = 0;
  int num_contacts = 0;

  while (time < time_end) {
    // If enabled, output data for PovRay postprocessing.
    if (sim_frame == next_out_frame) {
      cout << endl;
      cout << "---- Frame:          " << out_frame + 1 << endl;
      cout << "     Sim frame:      " << sim_frame << endl;
      cout << "     Time:           " << time << endl;
      cout << "     Speed:          " << vehicle->GetVehicle()->GetVehicleSpeed() << endl;
      cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
      cout << "     Execution time: " << exec_time << endl;

      if (povray_output) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
        utils::WriteShapesPovray(system, filename);
      }

      out_frame++;
      next_out_frame += out_steps;
      num_contacts = 0;
    }

    // Release the vehicle chassis at the end of the hold time.
    if (vehicle->GetVehicle()->GetChassis()->GetBodyFixed() && time > time_hold) {
      cout << endl << "Release vehicle t = " << time << endl;
      vehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
      for (int i = 0; i < 2 * vehicle->GetVehicle()->GetNumberAxles(); i++) {
        vehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
      }
    }

    // Update vehicle
    vehicle->Update(time);

// Advance dynamics.
#ifdef CHRONO_OPENGL
    if (gl_window.Active()) {
      gl_window.DoStepDynamics(time_step);
      gl_window.Render();
    } else
      break;
#else
    system->DoStepDynamics(time_step);
#endif

    progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);
    ////TimingOutput(system);

    // Periodically display maximum constraint violation
    if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
      std::vector<double> cvec;
      ////vehicle->GetVehicle()->LogConstraintViolations();
      cout << "  Max. violation = " << system->CalculateConstraintViolation(cvec) << endl;
    }

    // Update counters.
    time += time_step;
    sim_frame++;
    exec_time += system->GetTimerStep();
    num_contacts += system->GetNcontacts();
  }

  // Final stats
  cout << "==================================" << endl;
  cout << "Simulation time:   " << exec_time << endl;
  cout << "Number of threads: " << threads << endl;

  delete vehicle;
  delete tire_cb;

  return 0;
}
