// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demonstration of the moving patch option for the CRM terrain system in
// Chrono::Vehicle.
//
// =============================================================================

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;
using namespace chrono::vsg3d;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // ----------
    // Parameters
    // ----------

    double tend = 300;        // simulation end time
    double render_fps = 200;  // rendering FPS

    // Terrain dimensions
    double terrain_length = 0.4;
    double terrain_width = 0.16;
    double terrain_depth = 0.12;

    // Set CRM spacing
    double spacing = 0.04;

    // Moving patch settings
    double buffer_dist = 0.2;  // Look-ahead distance
    double shift_dist = 0.12;   // Patch shift distance

    // Camera location
    enum CameraType { FREE, TOP, FRONT, TRACK };
    CameraType cam_type = FREE;

    // -------------
    // Create system
    // -------------

    ChSystemNSC sysMBS;
    sysMBS.SetGravitationalAcceleration({0, 0, -9.81});
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ---------------
    // Create bodies
    // ---------------

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Tracked body parameters
    double kmh_to_ms = 1000.0 / 3600;
    double body_rad = 0.2;               // Radius (m)
    double body_speed = 5 * kmh_to_ms;  // Forward speed (m/s)

    ChVector3d pos(0, terrain_width / 2, terrain_depth + 1.5 * body_rad);
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1);
    body->SetInertiaXX(ChVector3d(1, 1, 1));
    body->SetPosDt(ChVector3d(body_speed, 0, 0));
    body->SetPos(pos);
    sysMBS.AddBody(body);

    auto body_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    utils::AddSphereGeometry(body.get(), body_mat, body_rad, ChVector3d(0, 0, 0));

    auto joint = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint->Initialize(ground, body, ChFrame<>(pos, QuatFromAngleY(CH_PI_2)));
    sysMBS.AddLink(joint);

    // ---------------------------------
    // Set solver and integrator for MBD
    // ---------------------------------

    double step_size = 0;
    ChSolver::Type solver_type;
    ChTimestepper::Type integrator_type;

    step_size = 5e-4;
    solver_type = ChSolver::Type::BARZILAIBORWEIN;
    integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;

    SetChronoSolver(sysMBS, solver_type, integrator_type, 1);
    sysMBS.SetNumThreads(1, 1, 1);

    // ------------------
    // Create the terrain
    // ------------------

    // CRM material properties
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    CRMTerrain terrain(sysMBS, spacing);
    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();
    terrain.SetVerbose(true);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

    // Set SPH parameters and soil material properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::ADAMI;
    ////sph_params.num_proximity_search_steps = 1;
    terrain.SetSPHParameters(sph_params);

    // No FSI bodies

    // Construct terrain, with moving patch feature
    terrain.ConstructMovingPatch({terrain_length, terrain_width, terrain_depth}, body, buffer_dist, shift_dist);

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // ------------------------
    // Initialize visualization
    // ------------------------

    auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(false);
    visFSI->EnableFlexBodyMarkers(false);
    visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f),
                                                                                       aabb.min.z(), aabb.max.z()));

    auto visVSG = chrono_types::make_shared<ChVisualSystemVSG>();
    visVSG->AttachSystem(&sysMBS);
    visVSG->AttachPlugin(visFSI);
    visVSG->ToggleAbsFrameVisibility();
    visVSG->SetWindowTitle("CRM moving patch demo");
    visVSG->SetCameraVertical(CameraVerticalDir::Z);
    visVSG->AddCamera(ChVector3d(terrain_length / 2, -3, 2), ChVector3d(terrain_length / 2, 0, 0));
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
    visVSG->EnableSkyBox();
    visVSG->SetCameraAngleDeg(40.0);
    visVSG->SetLightIntensity(1.0f);
    visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    visVSG->Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;

    cout << "Start simulation..." << endl;

    while (time < tend) {
        // Run-time visualization
        if (time >= render_frame / render_fps) {
            if (!visVSG->Run())
                break;

            switch (cam_type) {
                case TOP: {
                    ChVector3d cam_point = body->GetPos();
                    ChVector3d cam_loc = cam_point + ChVector3d(0, 0, 2);
                    visVSG->UpdateCamera(cam_loc, cam_point);
                    break;
                }
                case FRONT: {
                    double body_x = body->GetPos().x();
                    ChVector3d cam_loc(body_x + buffer_dist, -5, 0);
                    ChVector3d cam_point(body_x + buffer_dist, 0, 0);
                    visVSG->UpdateCamera(cam_loc, cam_point);
                    break;
                }
                case TRACK: {
                    ChVector3d cam_point = body->GetPos();
                    ChVector3d cam_loc = cam_point + ChVector3d(-4 * body_rad, -2, 0.5);
                    visVSG->UpdateCamera(cam_loc, cam_point);
                    break;
                }
                default:
                    break;
            }

            visVSG->Render();
            render_frame++;
        }

        // Synchronize terrain (performs moving patch logic)
        terrain.Synchronize(time);

        // Advance dynamics of multibody and fluid systems concurrently
        
        ////static bool moved = false;
        ////if (terrain.PatchMoved())
        ////    moved = true;
        ////if (!moved)
        ////    terrain.DoStepDynamics(step_size);
        
        terrain.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }

    return 0;
}
