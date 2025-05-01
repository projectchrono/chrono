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
// Demonstration of the moving patch option for the granular terrain sys in
// Chrono::Vehicle.
//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    double time_step = 5e-3;  // Integration time step (s)
    double time_pitch = 1;    // Time when gravity is rotated (s)
    double time_end = 10;     // Final simulation time (s)

    // Terrain parameters
    ChVector3d center(0, 0, 0);   // Center of initial patch
    double hdimX = 1.5;           // Length of patch
    double hdimY = 0.75;          // Width of patch
    unsigned int num_layers = 6;  // Requested number of layers
    bool rough = false;           // Fixed base layer?
    bool moving_patch = true;     // Enable moving patch feature?
    double buffer_dist = 2.0;     // Look-ahead distance (m)
    double shift_dist = 0.4;      // Patch shift distance (m)
    double slope = 30;            // Terrain slope (degrees)
    double radius = 20;           // Particle radius (mm)
    double rho = 2000;            // Granular material density (kg/m3)
    double mu = 0.9;              // Coefficient of friction
    double coh = 20;              // Cohesion pressure (kPa)

    // Convert terrain parameters
    double slope_g = slope * CH_DEG_TO_RAD;  // Slope (rad)
    double r_g = radius / 1000;              // Particle radius (m)
    double rho_g = rho;                      // Granular material density (kg/m3)
    double mu_g = mu;                        // Coefficient of friction
    double area = CH_PI * r_g * r_g;         // Particle cross-area (m2)
    double coh_force = area * (coh * 1e3);   // Cohesion force (N)
    double coh_g = coh_force * time_step;    // Cohesion impulse (Ns)

    // Tracked body parameters
    double kmh_to_ms = 1000.0 / 3600;
    double body_rad = 0.2;               // Radius (m)
    double body_speed = 50 * kmh_to_ms;  // Forward speed (m/s)

    // Collision envelope (10% of particle radius)
    double envelope = 0.1 * r_g;

    // Camera location
    enum CameraType { FIXED, FRONT, TRACK };
    CameraType cam_type = FRONT;

    // ----------------------------------
    // Create the multicore Chrono sys
    // ----------------------------------

    // Prepare rotated acceleration vector
    ChVector3d gravity(0, 0, -9.81);
    ChVector3d gravityR = ChMatrix33<>(slope_g, ChVector3d(0, 1, 0)) * gravity;

    ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->SetGravitationalAcceleration(gravity);

    // Set number of threads
    sys->SetNumThreads(4);

    // Edit sys settings
    sys->GetSettings()->solver.tolerance = 1e-3;
    sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys->GetSettings()->solver.max_iteration_normal = 0;
    sys->GetSettings()->solver.max_iteration_sliding = 50;
    sys->GetSettings()->solver.max_iteration_spinning = 0;
    sys->GetSettings()->solver.max_iteration_bilateral = 100;
    sys->GetSettings()->solver.compute_N = false;
    sys->GetSettings()->solver.alpha = 0;
    sys->GetSettings()->solver.cache_step_length = true;
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.contact_recovery_speed = 1000;
    sys->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    sys->ChangeSolverType(SolverType::BB);

    sys->GetSettings()->collision.collision_envelope = envelope;
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys->GetSettings()->collision.broadphase_grid = ChBroadphase::GridType::FIXED_RESOLUTION;
    sys->GetSettings()->collision.bins_per_axis = vec3(100, 30, 2);

    // ------------------
    // Create the terrain
    // ------------------

    GranularTerrain terrain(sys);
    auto mat = std::static_pointer_cast<ChContactMaterialNSC>(terrain.GetContactMaterial());
    mat->SetFriction((float)mu_g);
    mat->SetCohesion((float)coh_g);
    terrain.SetContactMaterial(mat);
    terrain.SetCollisionEnvelope(envelope / 5);
    if (rough) {
        int nx = (int)std::round((2 * hdimX) / (4 * r_g));
        int ny = (int)std::round((2 * hdimY) / (4 * r_g));
        terrain.EnableRoughSurface(nx, ny);
    }
    terrain.EnableVisualization(true);
    terrain.EnableVerbose(true);

    terrain.Initialize(center, 2 * hdimX, 2 * hdimY, num_layers, r_g, rho_g, ChVector3d(0, 0, -2));
    uint actual_num_particles = terrain.GetNumParticles();
    double terrain_height = terrain.GetHeight(ChVector3d(0, 0, 0));

    std::cout << "Number of particles: " << actual_num_particles << std::endl;
    std::cout << "Terrain height:      " << terrain_height << std::endl;

    // ---------------
    // Create the body
    // ---------------

    ChVector3d pos(terrain.GetPatchRear(), (terrain.GetPatchLeft() + terrain.GetPatchRight()) / 2,
                   terrain_height + 2 * body_rad);
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1);
    body->SetInertiaXX(ChVector3d(1, 1, 1));
    body->SetPosDt(ChVector3d(body_speed, 0, 0));
    body->SetPos(pos);
    sys->AddBody(body);

    auto body_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    utils::AddSphereGeometry(body.get(), body_mat, body_rad, ChVector3d(0, 0, 0));

    auto joint = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint->Initialize(terrain.GetGroundBody(), body, ChFrame<>(pos, QuatFromAngleY(CH_PI_2)));
    sys->AddLink(joint);

    // Enable moving patch, based on body location
    if (moving_patch)
        terrain.EnableMovingPatch(body, buffer_dist, shift_dist, ChVector3d(0, 0, -2));

    // ------------------------
    // Initialize visualization
    // ------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(sys);
    vis->SetWindowTitle("Granular terrain demo");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(center - ChVector3d(0, 3, 0), center);
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyBox();
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->Initialize();

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
            sys->SetGravitationalAcceleration(gravityR);
            is_pitched = true;
        }

        terrain.Synchronize(time);
        sys->DoStepDynamics(time_step);

        ////if (terrain.PatchMoved()) {
        ////    auto aabb_min = sys->data_manager->measures.collision.rigid_min_bounding_point;
        ////    auto aabb_max = sys->data_manager->measures.collision.rigid_max_bounding_point;
        ////    std::cout << "   Global AABB: " << std::endl;
        ////    std::cout << "   " << aabb_min.x << "  " << aabb_min.y << "  " << aabb_min.z << std::endl;
        ////    std::cout << "   " << aabb_max.x << "  " << aabb_max.y << "  " << aabb_max.z << std::endl;
        ////}

        if (vis->Run()) {
            switch (cam_type) {
                case FRONT: {
                    double body_x = body->GetPos().x();
                    ChVector3d cam_loc(body_x + buffer_dist, -4, 0);
                    ChVector3d cam_point(body_x + buffer_dist, 0, 0);
                    vis->UpdateCamera(cam_loc, cam_point);
                    break;
                }
                case TRACK: {
                    ChVector3d cam_point = body->GetPos();
                    ChVector3d cam_loc = cam_point + ChVector3d(-3 * body_rad, -1, 0.6);
                    vis->UpdateCamera(cam_loc, cam_point);
                    break;
                }
                default:
                    break;
            }
            vis->Render();
        } else {
            break;
        }

        time += time_step;
    }

    return 0;
}
