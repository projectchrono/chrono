// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Demonstration of the granular terrain system in Chrono::Vehicle.
//
// =============================================================================

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    double time_step = 5e-3;  // Integration time step (s)
    double time_pitch = 1;    // Time when gravity is rotated (s)
    double time_end = 10;     // Final simulation time (s)

    // Terrain parameters
    ChVector<> center(0, 0, 0);   // Center of initial patch
    double hdimX = 1.5;           // Length of patch
    double hdimY = 0.75;          // Width of patch
    unsigned int num_layers = 8;  // Requested number of layers
    bool rough = false;           // Fixed base layer?
    bool moving_patch = true;     // Enable moving patch feature?
    double slope = 0;             // Terrain slope (degrees)
    double radius = 20;           // Particle radius (mm)
    double rho = 2000;            // Granular material density (kg/m3)
    double mu = 0.9;              // Coefficient of friction
    double coh = 20;              // Cohesion pressure (kPa)

    // Convert terrain parameters
    double slope_g = slope * CH_C_DEG_TO_RAD;  // Slope (rad)
    double r_g = radius / 1000;                // Particle radius (m)
    double rho_g = rho;                        // Granular material density (kg/m3)
    double mu_g = mu;                          // Coefficient of friction
    double area = CH_C_PI * r_g * r_g;         // Particle cross-area (m2)
    double coh_force = area * (coh * 1e3);     // Cohesion force (N)
    double coh_g = coh_force * time_step;      // Cohesion impulse (Ns)

    // Tire parameters
    double tire_rad = 0.8;           // Radius (m)
    double tire_ang_vel = CH_C_2PI;  // Tire angular velocity (rad/s)

    // Collision envelope (10% of particle radius)
    double envelope = 0.1 * r_g;

    // Camera location
    enum CameraType { FIXED, FRONT, TRACK };
    CameraType cam_type = FIXED;

    // ---------------------------------
    // Create the parallel Chrono system
    // ---------------------------------

    // Prepare rotated acceleration vector
    ChVector<> gravity(0, 0, -9.81);
    ChVector<> gravityR = ChMatrix33<>(slope_g, ChVector<>(0, 1, 0)) * gravity;

    ChSystemParallelNSC* system = new ChSystemParallelNSC();
    system->Set_G_acc(gravity);

    // Set number of threads
    int threads = 4;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    system->SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    // Edit system settings
    system->GetSettings()->solver.tolerance = 1e-3;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = 0;
    system->GetSettings()->solver.max_iteration_sliding = 50;
    system->GetSettings()->solver.max_iteration_spinning = 0;
    system->GetSettings()->solver.max_iteration_bilateral = 100;
    system->GetSettings()->solver.compute_N = false;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.cache_step_length = true;
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.contact_recovery_speed = 1000;
    system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system->GetSettings()->min_threads = threads;
    system->ChangeSolverType(SolverType::BB);

    system->GetSettings()->collision.collision_envelope = envelope;
    system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);
    system->GetSettings()->collision.fixed_bins = true;

    // ------------------
    // Create the terrain
    // ------------------

    GranularTerrain terrain(system);
    terrain.SetContactFrictionCoefficient((float)mu_g);
    terrain.SetContactCohesion((float)coh_g);
    terrain.SetCollisionEnvelope(envelope / 5);
    if (rough) {
        int nx = (int)std::round((2 * hdimX) / (4 * r_g));
        int ny = (int)std::round((2 * hdimY) / (4 * r_g));
        terrain.EnableRoughSurface(nx, ny);
    }
    terrain.EnableVisualization(true);
    terrain.EnableVerbose(true);

    terrain.Initialize(center, 2 * hdimX, 2 * hdimY, num_layers, r_g, rho_g);
    uint actual_num_particles = terrain.GetNumParticles();
    double terrain_height = terrain.GetHeight(0, 0);

    std::cout << "Number of particles: " << actual_num_particles << std::endl;
    std::cout << "Terrain height:      " << terrain_height << std::endl;

    // ---------------
    // Create the tire
    // ---------------

    ChVector<> tire_center(terrain.GetPatchRear() + tire_rad, (terrain.GetPatchLeft() + terrain.GetPatchRight()) / 2,
                           terrain_height + 1.01 * tire_rad);
    auto body = std::shared_ptr<ChBody>(system->NewBody());
    body->SetMass(500);
    body->SetInertiaXX(ChVector<>(20, 20, 20));
    body->SetPos(tire_center);
    body->SetRot(Q_from_AngZ(CH_C_PI_2));
    system->AddBody(body);

    auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile("tractor_wheel.obj"));

    auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    body->AddAsset(trimesh_shape);

    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, VNULL, ChMatrix33<>(1), 0.01);
    body->GetCollisionModel()->BuildModel();

    ////utils::AddSphereGeometry(body.get(), tire_rad, ChVector<>(0, 0, 0));

    body->SetCollide(true);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    body->AddAsset(col);

    auto motor = std::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(std::make_shared<ChFunction_Ramp>(0, -tire_ang_vel));
    motor->Initialize(body, terrain.GetGroundBody(), ChFrame<>(tire_center, Q_from_AngX(CH_C_PI_2)));
    system->Add(motor);

    std::cout << "Tire location: " << tire_center.x() << " " << tire_center.y() << " " << tire_center.z() << std::endl;

    // Enable moving patch, based on tire location
    if (moving_patch)
        terrain.EnableMovingPatch(body, 2.0, 0.2);

    // -----------------
    // Initialize OpenGL
    // -----------------

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Granular terrain demo", system);
    gl_window.SetCamera(center - ChVector<>(0, 3, 0), center, ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::SOLID);

    // ---------------
    // Simulation loop
    // ---------------

    bool is_pitched = false;
    double time = 0;

    while (time < time_end) {
        // Rotate gravity vector
        if (!is_pitched && time > time_pitch) {
            std::cout << time << "    Pitch: " << gravityR.x() << " " << gravityR.y() << " " << gravityR.z()
                      << std::endl;
            system->Set_G_acc(gravityR);
            is_pitched = true;
        }

        terrain.Synchronize(time);
        system->DoStepDynamics(time_step);

        ////if (terrain.PatchMoved()) {
        ////    auto aabb_min = system->data_manager->measures.collision.rigid_min_bounding_point;
        ////    auto aabb_max = system->data_manager->measures.collision.rigid_max_bounding_point;
        ////    std::cout << "   Global AABB: " << std::endl;
        ////    std::cout << "   " << aabb_min.x << "  " << aabb_min.y << "  " << aabb_min.z << std::endl;
        ////    std::cout << "   " << aabb_max.x << "  " << aabb_max.y << "  " << aabb_max.z << std::endl;
        ////}

        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (gl_window.Active()) {
            switch (cam_type) {
                case FRONT: {
                    ChVector<> cam_loc(terrain.GetPatchFront(), -3, 0);
                    ChVector<> cam_point(terrain.GetPatchFront(), 0, 0);
                    gl_window.SetCamera(cam_loc, cam_point, ChVector<>(0, 0, 1), 0.05f);
                    break;
                }
                case TRACK: {
                    ChVector<> cam_point = body->GetPos();
                    ChVector<> cam_loc = cam_point + ChVector<>(-3 * tire_rad, -1, 0.6);
                    gl_window.SetCamera(cam_loc, cam_point, ChVector<>(0, 0, 1), 0.05f);
                    break;
                }
                default:
                    break;
            }
            gl_window.Render();
        } else {
            break;
        }

        time += time_step;
    }

    return 0;
}
