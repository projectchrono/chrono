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
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

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
double test_tolerance = 1e-8;

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
float contact_recovery_speed = 10e30;
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
  ChSharedPtr<ChMaterialSurface> mat_walls(new ChMaterialSurface);
  mat_walls->SetFriction(mu_walls);

  ChSharedPtr<ChBody> container(new ChBody(
#ifndef BULLET
      new ChCollisionModelParallel
#endif
      ));
  container->SetMaterialSurface(mat_walls);
  container->SetIdentifier(Id_container);
  container->SetBodyFixed(true);
  container->SetCollide(true);
  container->SetMass(10000.0);

  // Attach geometry of the containing bin
  container->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(container.get_ptr(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick));
  utils::AddBoxGeometry(container.get_ptr(), ChVector<>(hthick, hdimY, hdimZ), ChVector<>(-hdimX - hthick, 0, hdimZ));
  utils::AddBoxGeometry(container.get_ptr(), ChVector<>(hthick, hdimY, hdimZ), ChVector<>(hdimX + hthick, 0, hdimZ));
  utils::AddBoxGeometry(container.get_ptr(), ChVector<>(hdimX, hthick, hdimZ), ChVector<>(0, -hdimY - hthick, hdimZ));
  utils::AddBoxGeometry(container.get_ptr(), ChVector<>(hdimX, hthick, hdimZ), ChVector<>(0, hdimY + hthick, hdimZ));
  container->GetCollisionModel()->BuildModel();

  system->AddBody(container);

  ChSharedPtr<ChBody> ground(new ChBody(
#ifndef BULLET
      new ChCollisionModelParallel
#endif
      ));
  ground->SetMaterialSurface(mat_walls);
  ground->SetIdentifier(Id_ground);
  ground->SetBodyFixed(true);
  ground->SetCollide(true);
  ground->SetMass(1.0);

  system->AddBody(ground);
}

void CreateGranularMaterial(ChSystemParallel* sys) {
  // Common material
  ChSharedPtr<ChMaterialSurface> ballMat(new ChMaterialSurface);
  ballMat->SetFriction(.5);

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

        ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
        ball->SetMaterialSurface(ballMat);

        ball->SetIdentifier(ballId++);
        ball->SetMass(mass);
        ball->SetInertiaXX(inertia);
        ball->SetPos(pos + rnd);
        ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
        ball->SetBodyFixed(false);
        ball->SetCollide(true);

        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get_ptr(), radius);
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
  msystem->GetSettings()->solver.solver_mode = SLIDING;
  msystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
  msystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
  msystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
  msystem->GetSettings()->solver.alpha = 0;
  msystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
  msystem->ChangeSolverType(APGD);
  msystem->GetSettings()->collision.collision_envelope = 0.00;
  msystem->GetSettings()->collision.bins_per_axis = I3(10, 10, 10);
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
bool CompareContacts(ChSystemParallel* msystem) {
  real3* norm = msystem->data_manager->host_data.norm_rigid_rigid.data();
  real3* ptA = msystem->data_manager->host_data.cpta_rigid_rigid.data();
  real3* ptB = msystem->data_manager->host_data.cptb_rigid_rigid.data();
  real3* pos_data = msystem->data_manager->host_data.pos_rigid.data();
  chrono::int2* ids = msystem->data_manager->host_data.bids_rigid_rigid.data();
  real4* rot = msystem->data_manager->host_data.rot_rigid.data();

  ((ChLcpSolverParallelDVI*)msystem->GetLcpSolverSpeed())->ComputeD();

  CompressedMatrix<real>& D_n_T = msystem->data_manager->host_data.D_n_T;
  CompressedMatrix<real>& D_t_T = msystem->data_manager->host_data.D_t_T;
  CompressedMatrix<real>& D_s_T = msystem->data_manager->host_data.D_s_T;

  int nnz_normal = 6 * 2 * msystem->data_manager->num_rigid_contacts;
  int nnz_tangential = 6 * 4 * msystem->data_manager->num_rigid_contacts;
  int nnz_spinning = 6 * 3 * msystem->data_manager->num_rigid_contacts;

  //  StrictEqual(D_n_T.nonZeros(), nnz_normal);
  //  StrictEqual(D_t_T.nonZeros(), nnz_tangential);
  //  StrictEqual(D_s_T.nonZeros(), nnz_spinning);

  cout << D_n_T.nonZeros() << " " << nnz_normal << endl;
  cout << D_t_T.nonZeros() << " " << nnz_tangential << endl;
  cout << D_s_T.nonZeros() << " " << nnz_spinning << endl;

  //#pragma omp parallel for
  for (int index = 0; index < msystem->data_manager->num_rigid_contacts; index++) {
    real3 U = norm[index], V, W;
    real3 T3, T4, T5, T6, T7, T8;
    real3 TA, TB, TC;
    real3 TD, TE, TF;

    Orthogonalize(U, V, W);

    chrono::int2 body_id = ids[index];

    int row = index;

    // The position is subtracted here now instead of performing it in the narrowphase
    Compute_Jacobian(rot[body_id.x], U, V, W, ptA[index] - pos_data[body_id.x], T3, T4, T5);
    Compute_Jacobian(rot[body_id.y], U, V, W, ptB[index] - pos_data[body_id.y], T6, T7, T8);

    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 0), -U.x);
    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 1), -U.y);
    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 2), -U.z);

    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 3), T3.x);
    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 4), T3.y);
    StrictEqual(D_n_T(row * 1 + 0, body_id.x * 6 + 5), T3.z);

    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 0), U.x);
    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 1), U.y);
    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 2), U.z);

    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 3), -T6.x);
    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 4), -T6.y);
    StrictEqual(D_n_T(row * 1 + 0, body_id.y * 6 + 5), -T6.z);

    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 0), -V.x);
    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 1), -V.y);
    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 2), -V.z);

    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 3), T4.x);
    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 4), T4.y);
    StrictEqual(D_t_T(row * 2 + 0, body_id.x * 6 + 5), T4.z);

    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 0), -W.x);
    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 1), -W.y);
    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 2), -W.z);

    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 3), T5.x);
    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 4), T5.y);
    StrictEqual(D_t_T(row * 2 + 1, body_id.x * 6 + 5), T5.z);

    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 0), V.x);
    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 1), V.y);
    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 2), V.z);

    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 3), -T7.x);
    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 4), -T7.y);
    StrictEqual(D_t_T(row * 2 + 0, body_id.y * 6 + 5), -T7.z);

    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 0), W.x);
    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 1), W.y);
    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 2), W.z);

    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 3), -T8.x);
    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 4), -T8.y);
    StrictEqual(D_t_T(row * 2 + 1, body_id.y * 6 + 5), -T8.z);
  }

  return true;
}

int main(int argc, char* argv[]) {
  CHOMPfunctions::SetNumThreads(1);

  // No animation by default (i.e. when no program arguments)
  bool animate = (argc > 1);

  ChSystemParallelDVI* msystem = new ChSystemParallelDVI();

  SetupSystem(msystem);

#ifdef BULLET
  msystem->ChangeCollisionSystem(COLLSYS_BULLET_PARALLEL);
#else
  msystem->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_MPR;
#endif

  // Initialize counters
  double time = 0;
  int sim_frame = 0;
  double exec_time = 0;
  int num_contacts = 0;
  double time_end = time_settling_max;

  if (animate) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Narrowphase", msystem);
    gl_window.SetCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Loop until reaching the end time...
    while (time < time_end) {
      if (gl_window.Active()) {
        // gl_window.DoStepDynamics(time_step);
        gl_window.Render();
      }
      msystem->DoStepDynamics(time_step);
      CompareContacts(msystem);
      cout << "Time: " << time << endl;
      time += time_step;
    }

#else
    std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
    return false;
#endif
  } else {
    while (time < time_end) {
      msystem->DoStepDynamics(time_step);

      cout << "Time: " << time << endl;
      time += time_step;
    }
  }

  return 0;
}
