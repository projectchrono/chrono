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
// Author: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test to compare solutions from different narrowphase
// algorithms
// The global reference frame has Z up.
// All units SI (CGS, i.e., centimeter - gram - second)
//
// =============================================================================

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "unit_testing.h"

// Comment the following line to use parallel collision detection
//#define BULLET

// Control use of OpenGL run-time rendering
//#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::flush;
using std::endl;

// -----------------------------------------------------------------------------
// Global problem definitions
// -----------------------------------------------------------------------------
// Tolerance for test
double test_tolerance = 1e-5;

// Save PovRay post-processing data?
bool write_povray_data = true;

// Load the bodies from a checkpoint file?
bool loadCheckPointFile = false;

// Simulation times
double time_settling_min = 0.1;
double time_settling_max = 1.0;

// Stopping criteria for settling (fraction of particle radius)
double settling_tol = 0.2;

// Solver settings
double time_step = 1e-3;
int max_iteration_normal = 0;
int max_iteration_sliding = 25;
int max_iteration_spinning = 0;
float contact_recovery_speed = 10e30f;
double tolerance = 1e-2;

// Simulation frame at which detailed timing information is printed
int timing_frame = -1;

// Gravitational acceleration [m/s^2]
double gravity = 9.81;

// Parameters for the mechanism
int Id_container = 0;  // body ID for the containing bin
int Id_ground = 1;     // body ID for the ground

double hdimX = 2.0 / 2;   // [m] bin half-length in x direction
double hdimY = 2.0 / 2;   // [m] bin half-depth in y direction
double hdimZ = 2.0 / 2;   // [m] bin half-height in z direction
double hthick = 0.1 / 2;  // [m] bin half-thickness of the walls
float mu_walls = 0.3f;

// Parameters for the granular material
int Id_g = 2;         // start body ID for particles
double r_g = 0.1;     // [m] radius of granular sphers
double rho_g = 1000;  // [kg/m^3] density of granules
float mu_g = 0.5f;

void CreateMechanismBodies(ChSystemParallel* system) {
  auto mat_walls = std::make_shared<ChMaterialSurface>();
  mat_walls->SetFriction(mu_walls);

  std::shared_ptr<ChBody> container(system->NewBody());
  container->SetMaterialSurface(mat_walls);
  container->SetIdentifier(Id_container);
  container->SetBodyFixed(true);
  container->SetCollide(true);
  container->SetMass(10000.0);

  // Attach geometry of the containing bin
  container->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick));
  utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ), ChVector<>(-hdimX - hthick, 0, hdimZ));
  utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ), ChVector<>(hdimX + hthick, 0, hdimZ));
  utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ), ChVector<>(0, -hdimY - hthick, hdimZ));
  utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ), ChVector<>(0, hdimY + hthick, hdimZ));
  container->GetCollisionModel()->BuildModel();

  system->AddBody(container);

  std::shared_ptr<ChBody> ground(system->NewBody());
  ground->SetMaterialSurface(mat_walls);
  ground->SetIdentifier(Id_ground);
  ground->SetBodyFixed(true);
  ground->SetCollide(true);
  ground->SetMass(1.0);

  system->AddBody(ground);
}

void CreateGranularMaterial(ChSystemParallel* sys) {
  // Common material
  auto ballMat = std::make_shared<ChMaterialSurface>();
  ballMat->SetFriction(1.0);

  // Create the falling balls
  int ballId = 0;
  double mass = 1;
  double radius = 0.15;
  ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);
  srand(1);

  for (int ix = -2; ix < 3; ix++) {
    for (int iy = -2; iy < 3; iy++) {
      for (int iz = -2; iz < 3; iz++) {
        ChVector<> rnd(rand() % 1000 / 100000.0, rand() % 1000 / 100000.0, rand() % 1000 / 100000.0);
        ChVector<> pos(0.4 * ix, 0.4 * iy, 0.4 * iz + 1);

        std::shared_ptr<ChBody> ball(sys->NewBody());
        ball->SetMaterialSurface(ballMat);

        ball->SetIdentifier(ballId++);
        ball->SetMass(mass);
        ball->SetInertiaXX(inertia);
        ball->SetPos(pos + rnd);
        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
        ball->SetBodyFixed(false);
        ball->SetCollide(true);

        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), radius);
        ball->GetCollisionModel()->BuildModel();

        sys->AddBody(ball);
      }
    }
  }
}
// =============================================================================

void SetupSystem(ChSystemParallelDVI* msystem) {
  msystem->Set_G_acc(ChVector<>(0, 0, -gravity));

  msystem->GetSettings()->solver.tolerance = tolerance;
  msystem->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
  msystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
  msystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
  msystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
  msystem->GetSettings()->solver.alpha = 0;
  msystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
  msystem->SetMaxPenetrationRecoverySpeed(contact_recovery_speed);
  msystem->ChangeSolverType(SolverType::APGD);
  msystem->GetSettings()->collision.collision_envelope = 0;
  msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
  CHOMPfunctions::SetNumThreads(1);
  msystem->GetSettings()->max_threads = 1;
  msystem->GetSettings()->perform_thread_tuning = false;

  // Create the mechanism bodies (all fixed).
  CreateMechanismBodies(msystem);

  // Create granular material.
  CreateGranularMaterial(msystem);
}
// Sync the positions and velocities of the rigid bodies
void Sync(ChSystemParallel* msystem_A, ChSystemParallel* msystem_B) {
  for (int i = 0; i < msystem_A->Get_bodylist()->size(); i++) {
    ChVector<> pos = msystem_B->Get_bodylist()->at(i)->GetPos();
    ChVector<> pos_dt = msystem_B->Get_bodylist()->at(i)->GetPos_dt();
    msystem_A->Get_bodylist()->at(i)->SetPos(pos);
    msystem_A->Get_bodylist()->at(i)->SetPos_dt(pos_dt);
  }
}
bool CompareContacts(ChSystemParallel* msystem_A, ChSystemParallel* msystem_B) {
  bool passing = true;
  int num_contacts_A = msystem_A->data_manager->num_rigid_contacts;
  int num_contacts_B = msystem_B->data_manager->num_rigid_contacts;
  cout << "Number of contacts mpr: " << num_contacts_A << " r: " << num_contacts_B << endl;

  if (num_contacts_A != num_contacts_B) {
    std::cout << num_contacts_A << " does not equal " << num_contacts_B << std::endl;
    passing = false;
  }

  // compare depth
  if (passing) {
    cout << "Compare Depth" << endl;
    for (int i = 0; i < num_contacts_A; i++) {
      real depth_A = msystem_A->data_manager->host_data.dpth_rigid_rigid[i];
      real depth_B = msystem_B->data_manager->host_data.dpth_rigid_rigid[i];

      if (fabs(depth_A - depth_B) > test_tolerance) {
        std::cout << depth_A << " does not equal " << depth_B << " " << fabs(depth_A - depth_B) << std::endl;
        passing = false;
      }
    }
  }
  if (passing) {
    cout << "Compare Normals" << endl;
    for (int i = 0; i < num_contacts_A; i++) {
      real3 norm_A = msystem_A->data_manager->host_data.norm_rigid_rigid[i];
      real3 norm_B = msystem_B->data_manager->host_data.norm_rigid_rigid[i];
      real x = norm_A.x;
      real y = norm_B.x;

      if (fabs(x - y) > test_tolerance) {
        std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
        passing = false;
      }
      x = norm_A.y;
      y = norm_B.y;
      if (fabs(x - y) > test_tolerance) {
        std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
        passing = false;
      }
      x = norm_A.z;
      y = norm_B.z;
      if (fabs(x - y) > test_tolerance) {
        std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
        passing = false;
      }
    }
  }
  //  if (passing) {
  //    cout << "Compare Contact Points" << endl;
  //    for (int i = 0; i < num_contacts_A; i++) {
  //      real3 pta_A = msystem_A->data_manager->host_data.cpta_rigid_rigid[i];
  //      real3 pta_B = msystem_B->data_manager->host_data.cpta_rigid_rigid[i];
  //      WeakEqual(pta_A, pta_B, test_tolerance);
  //      real3 ptb_A = msystem_A->data_manager->host_data.cptb_rigid_rigid[i];
  //      real3 ptb_B = msystem_B->data_manager->host_data.cptb_rigid_rigid[i];
  //      WeakEqual(ptb_A, ptb_B, test_tolerance);
  //    }
  //  }
  //  if (!passing) {
  //
  //    cout << "MPR:" << endl;
  //    for (int i = 0; i < num_contacts_A; i++) {
  //      vec2 id = msystem_A->data_manager->host_data.bids_rigid_rigid[i];
  //      real depth = msystem_A->data_manager->host_data.dpth_rigid_rigid[i];
  //      cout << id.x << " " << id.y << " " << depth << endl;
  //    }
  //    cout << "R:" << endl;
  //    for (int i = 0; i < num_contacts_B; i++) {
  //      vec2 id = msystem_B->data_manager->host_data.bids_rigid_rigid[i];
  //      real depth = msystem_B->data_manager->host_data.dpth_rigid_rigid[i];
  //      cout << id.x << " " << id.y << " " << depth << endl;
  //    }
  //
  //    // exit(1);
  //  }

  return true;
}

int main(int argc, char* argv[]) {
  // No animation by default (i.e. when no program arguments)
  bool animate = (argc > 1);

  ChSystemParallelDVI* msystem_mpr = new ChSystemParallelDVI();
  ChSystemParallelDVI* msystem_r = new ChSystemParallelDVI();

#ifdef BULLET
  msystem_mpr->ChangeCollisionSystem(CollisionSystemType::COLLSYS_BULLET_PARALLEL);
  msystem_r->ChangeCollisionSystem(CollisionSystemType::COLLSYS_BULLET_PARALLEL);
#endif

  SetupSystem(msystem_mpr);
  SetupSystem(msystem_r);

  // Edit system settings

  msystem_mpr->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_MPR;
  msystem_r->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;

  // Initialize counters
  double time = 0;
  int sim_frame = 0;
  double exec_time = 0;
  int num_contacts = 0;
  double time_end = time_settling_max;

  if (animate) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Narrowphase", msystem_mpr);
    gl_window.SetCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Loop until reaching the end time...
    while (time < time_end) {
      if (gl_window.Active()) {
        gl_window.Render();
      }
      msystem_mpr->DoStepDynamics(time_step);
      msystem_r->DoStepDynamics(time_step);
      Sync(msystem_mpr, msystem_r);
      CompareContacts(msystem_mpr, msystem_r);

      cout << "Time: " << time << endl;
      time += time_step;
    }

#else
    std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
    return false;
#endif
  } else {
    while (time < time_end) {
      msystem_mpr->DoStepDynamics(time_step);
      msystem_r->DoStepDynamics(time_step);
      Sync(msystem_mpr, msystem_r);
      CompareContacts(msystem_mpr, msystem_r);

      cout << "Time: " << time << endl;
      time += time_step;
    }
  }

  return 0;
}
