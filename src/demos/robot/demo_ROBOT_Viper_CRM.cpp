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
// Viper rover on CRM terrain (initialized from heightmap image)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/robot/viper/Viper.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::viper;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// CRM terrain patch type
enum class PatchType { RECTANGULAR, HEIGHT_MAP };
PatchType patch_type = PatchType::HEIGHT_MAP;

// Terrain dimensions (for RECTANGULAR or HEIGHT_MAP patch type)
double terrain_length = 12;
double terrain_width = 3;

// ===================================================================================================================

int main(int argc, char* argv[]) {
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    double tend = 30;
    double step_size = 5e-4;
    ChVector3d active_box_dim(0.6, 0.6, 0.6);

    bool render = true;       // use run-time visualization
    double render_fps = 200;  // rendering FPS

    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create rover
    cout << "Create rover..." << endl;
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    ChContactMaterialData wheel_mat(0.4f,   // mu
                                    0.2f,   // cr
                                    2e7f,   // Y
                                    0.3f,   // nu
                                    2e5f,   // kn
                                    40.0f,  // gn
                                    2e5f,   // kt
                                    20.0f   // gt
    );
    ChVector3d init_loc(1.25, 0.0, 0.55);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the CRM terrain system
    double initial_spacing = 0.03;
    CRMTerrain terrain(sys, initial_spacing);
    auto sysFSI = terrain.GetFsiSystemSPH();
    auto sysSPH = terrain.GetFluidSystemSPH();
    sysSPH->EnableCudaErrorCheck(false);
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

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

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.free_surface_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.use_variable_time_step = true;  // This makes the step size irrelevant - now we just make sure we are
                                               // within the max of exchange_info (meta step)
    terrain.SetSPHParameters(sph_params);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add rover wheels as FSI bodies
    cout << "Create wheel BCE markers..." << endl;
    std::string mesh_filename = GetChronoDataFile("robot/viper/obj/viper_cylwheel.obj");
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_filename, VNULL));

    //// TODO: FIX ChFsiProblemSPH to allow rotating geometry on body!!
    for (int i = 0; i < 4; i++) {
        auto wheel_body = rover->GetWheels()[i]->GetBody();
        auto yaw = (i % 2 == 0) ? QuatFromAngleZ(CH_PI) : QUNIT;
        terrain.AddRigidBody(wheel_body, geometry, false);
    }

    terrain.SetActiveDomain(active_box_dim);

    // Construct the terrain
    cout << "Create terrain..." << endl;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            // Create a rectangular terrain patch
            terrain.Construct({terrain_length, terrain_width, 0.25},  // length X width X height
                              ChVector3d(terrain_length / 2, 0, 0),   // patch center
                              BoxSide::ALL & ~BoxSide::Z_POS          // all boundaries, except top
            );
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a heigh field map image
            terrain.Construct(vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),  // height map image file
                              terrain_length, terrain_width,                           // length (X) and width (Y)
                              {0.25, 0.55},                                            // height range
                              0.25,                                                    // depth
                              true,                                                    // uniform depth
                              ChVector3d(terrain_length / 2, 0, 0),                    // patch center
                              BoxSide::Z_NEG                                           // bottom wall
            );
            break;
    }

    // Initialize the terrain system
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Create run-time visualization
    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(aabb.min.z(), aabb.max.z());

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::BROWN);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Viper rover on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(init_loc + ChVector3d(0, 6, 0.5), init_loc);
        visVSG->SetLightIntensity(0.9f);

        visVSG->Initialize();
        vis = visVSG;
    }

    // Start the simulation
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;
    double exchange_info = 5 * step_size;
    while (time < tend) {
        rover->Update();

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Advance dynamics of multibody and fluid systems concurrently
        terrain.DoStepDynamics(exchange_info);

        time += exchange_info;
        sim_frame++;
    }

    terrain.PrintStats();
    std::string out_dir = GetChronoOutputPath() + "ROBOT_Viper_CRM/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    terrain.PrintTimeSteps(out_dir + "time_steps.txt");

    return 0;
}
