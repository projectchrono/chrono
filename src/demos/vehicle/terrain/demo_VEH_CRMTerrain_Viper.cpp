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

#include "chrono_fsi/ChSystemFsi.h"

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

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::viper;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.7;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    double tend = 30;
    double step_size = 5e-4;
    ChVector3d active_box_hdim(0.4, 0.3, 0.5);

    bool visualization = true;             // run-time visualization
    double visualizationFPS = 0;           // frames rendered per second (0: every frame)
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
    ChVector3d init_loc(-1, 0.0, 2.4);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the CRM terrain system
    double initial_spacing = 0.03;
    CRMTerrain terrain(sys, initial_spacing);
    terrain.SetVerbose(verbose);
    ChSystemFsi& sysFSI = terrain.GetSystemFSI();

    // Set SPH parameters and soil material properties
    const ChVector3d gravity(0, 0, -9.81);
    sysFSI.SetGravitationalAcceleration(gravity);
    sys.SetGravitationalAcceleration(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_PI / 10;  // default
    mat_props.dilation_angle = CH_PI / 10;  // default
    mat_props.cohesion_coeff = cohesion;

    sysFSI.SetElasticSPH(mat_props);

    ChSystemFsi::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.kernel_h = initial_spacing;
    sph_params.initial_spacing = initial_spacing;
    sph_params.kernel_threshold = 0.8;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;

    sysFSI.SetSPHParameters(sph_params);
    sysFSI.SetStepSize(step_size);

    sysFSI.SetActiveDomain(ChVector3d(active_box_hdim));

    sysFSI.SetOutputLength(0);

    // Add rover wheels as FSI bodies
    cout << "Create wheel BCE markers..." << endl;
    std::string mesh_filename = GetChronoDataFile("robot/viper/obj/viper_wheel.obj");
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL));

    //// TODO: FIX ChFsiProblem to allow rotating geometry on body!!
    for (int i = 0; i < 4; i++) {
        auto wheel_body = rover->GetWheels()[i]->GetBody();
        auto yaw = (i % 2 == 0) ? QuatFromAngleZ(CH_PI) : QUNIT;
        terrain.AddRigidBody(wheel_body, geometry, false);
    }

    // Construct the terrain
    cout << "Create terrain..." << endl;
    terrain.Construct(vehicle::GetDataFile("terrain/height_maps/terrain3.bmp"),  // height map image file
                      4, 4,                                                      // length (X) and width (Y)
                      {0, 2.55},                                                 // height range
                      0.3,                                                       // depth
                      true,                                                      // uniform depth
                      ChVector3d(0, 0, 0),                                       // patch center
                      true,                                                      // bottom wall?
                      false                                                      // side walls?
    );



    // Initialize the terrain system
    cout << "Initialize CRM terrain..." << endl;
    terrain.Initialize();

    auto aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << sysFSI.GetNumFluidMarkers() << endl;
    cout << "  Bndry BCE markers: " << sysFSI.GetNumBoundaryMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

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
    if (visualization) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Viper rover on CRM deformable terrain");
        visFSI->SetVerbose(verbose);
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(init_loc + ChVector3d(0, 6, 0.5), init_loc);
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb.min.z(), aabb.max.z()));
        visFSI->AttachSystem(&sys);
        visFSI->Initialize();
    }

    // Start the simulation
    int render_steps = (visualizationFPS > 0) ? (int)std::round((1.0 / visualizationFPS) / step_size) : 1;
    double t = 0;
    int frame = 0;

    while (t < tend) {
        rover->Update();

        // Run-time visualization
        if (visualization && frame % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }
        if (!visualization) {
            std::cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRTF() << std::endl;
        }

        sysFSI.DoStepDynamics_FSI();
        t += step_size;
        frame++;
    }

    return 0;
}
