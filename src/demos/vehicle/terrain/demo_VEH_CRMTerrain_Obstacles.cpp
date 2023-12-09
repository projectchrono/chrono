// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Demonstration of the CRM terrain system in Chrono::Vehicle.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

const std::string out_dir = GetChronoOutputPath() + "SPH_TERRAIN_OBSTACLE";

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double density = 1700;
    double cohesion = 1e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    double tend = 30;
    double step_size = 5e-4;

    bool run_time_vis = true;              // run-time visualization?
    double run_time_vis_fps = 0;           // render frequency (0: every simulation frame)
    bool run_time_vis_terrain_sph = true;  // render terrain SPH particles?
    bool run_time_vis_terrain_bce = true;  // render terrain BCE markers?
    bool run_time_vis_bce = false;         // render moving BCE markers?

    bool verbose = true;

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain system
    CRMTerrain terrain(sys, 0.02);
    terrain.SetVerbose(verbose);
    ChSystemFsi& sysFSI = terrain.GetSystemFSI();

    // Set SPH parameters and soil material properties
    const ChVector<> gravity(0, 0, -9.81);
    sysFSI.Set_G_acc(gravity);
    sys.Set_G_acc(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_C_PI / 10;  // default
    mat_props.dilation_angle = CH_C_PI / 10;  // default
    mat_props.cohesion_coeff = 0;             // default
    mat_props.kernel_threshold = 0.8;

    sysFSI.SetElasticSPH(mat_props);
    sysFSI.SetDensity(density);
    sysFSI.SetCohesionForce(cohesion);

    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);

    // Add obstacles
    terrain.AddRigidObstacle(GetChronoDataFile("models/sphere.obj"), 0.25, 5000, ChContactMaterialData(),
                             ChFrame<>(ChVector<>(0, 0, 0.35)));

    // Terrain constructed from height map
    terrain.Construct(vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),  // height map image file
                      2, 1,                                                    // length (X) and width (Y)
                      {0, 0.3},                                                // height range
                      0.3,                                                     // depth
                      3,                                                       // number of BCE layers
                      ChVector<>(0, 0, 0),                                     // patch center
                      0.0,                                                     // patch yaw rotation
                      false                                                    // side walls?
    );

    // Rectangular terrain patch
    ////terrain.Construct(2, 1,                 // length (X) and width (Y)
    ////                  0.3,                  // depth
    ////                  3,                    // number of BCE layers
    ////                  ChVector<>(0, 0, 0),  // patch center
    ////                  0.0,                  // patch yaw rotation
    ////                  true                  // side walls?
    ////);

    terrain.SaveMarkers(out_dir);

    terrain.Initialize();

    // Create run-time visualization
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (run_time_vis) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI, verbose);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI, verbose);
#endif
                break;
            }
        }

        visFSI->SetTitle("CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector<>(2, 1, 0.5), ChVector<>(0, 0, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(run_time_vis_terrain_sph);
        visFSI->EnableBoundaryMarkers(run_time_vis_terrain_bce);
        visFSI->EnableRigidBodyMarkers(run_time_vis_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<HeightColorCallback>(-0.3, 0.3));
        visFSI->AttachSystem(&sys);
        visFSI->Initialize();
    }

    // Simulation loop
    int render_steps = (run_time_vis_fps > 0) ? (int)std::round((1.0 / run_time_vis_fps) / step_size) : 1;

    double t = 0;
    int frame = 0;

    while (t < tend) {
        if (run_time_vis && frame % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        sysFSI.DoStepDynamics_FSI();

        t += step_size;
        frame++;
    }

    return 0;
}
