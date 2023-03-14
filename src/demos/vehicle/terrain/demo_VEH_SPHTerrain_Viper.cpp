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
// Viper rover on SPH terrain (initialized from heightmap image)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/robot/viper/Viper.h"

#include "chrono_vehicle/terrain/SPHTerrain.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBoxShape.h"
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
using namespace chrono::geometry;
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

    double target_speed = 7.0;
    double tend = 30;
    double step_size = 5e-4;
    ChVector<> active_box_hdim(0.4, 0.3, 0.5);

    bool visualization = true;             // run-time visualization
    double visualizationFPS = 0;           // frames rendered per second (0: every frame)
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers
    bool chase_cam = true;                 // chase-cam or fixed camera

    bool verbose = true;

    // Create the Chrono system
    ChSystemNSC sys;

    // Create the SPH terrain system
    SPHTerrain terrain(sys, 0.01);
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

    sysFSI.SetActiveDomain(ChVector<>(active_box_hdim));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ADAMI);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);

    sysFSI.SetOutputLength(0);

    // Set the simulation domain size
    //// TODO: needed?
    ////sysFSI.SetContainerDim(ChVector<>(4, 2, 0.1));

    cout << "Create terrain..." << endl;
    terrain.Construct(vehicle::GetDataFile("terrain/height_maps/terrain3.bmp"),  // height map image file
                      4, 4,                                                      // length (X) and width (Y)
                      {0, 2.55},                                                 // height range
                      0.3,                                                       // depth
                      3,                                                         // number of BCE layers
                      ChVector<>(0, 0, 0),                                       // patch center
                      0.0,                                                       // patch yaw rotation
                      false                                                      // side walls?
    );

    // Create rover
    cout << "Create rover..." << endl;
    ViperWheelType wheel_type = ViperWheelType::RealWheel;
    std::string wheel_obj = "robot/viper/obj/viper_wheel.obj";
    ChContactMaterialData wheel_mat(0.4f,   // mu
                                    0.2f,   // cr
                                    2e7f,   // Y
                                    0.3f,   // nu
                                    2e5f,   // kn
                                    40.0f,  // gn
                                    2e5f,   // kt
                                    20.0f   // gt
    );
    ChVector<> init_loc(-1, 0.0, 2.4);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto rover = chrono_types::make_shared<Viper>(&sys, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(wheel_mat.CreateMaterial(sys.GetContactMethod()));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the wheel BCE markers
    cout << "Create wheel BCE markers..." << endl;
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    std::vector<ChVector<>> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, sysFSI.GetInitialSpacing(), BCE_wheel);

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        auto wheel_body = rover->GetWheels()[i]->GetBody();
        auto yaw = (i % 2 == 0) ? Q_from_AngZ(CH_C_PI) : QUNIT;
        sysFSI.AddFsiBody(wheel_body);
        sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, yaw), true);
    }

    // Initialize the terrain system
    cout << "Initialize SPH terrain..." << endl;
    terrain.Initialize();

    ChVector<> aabb_min, aabb_max;
    terrain.GetAABB(aabb_min, aabb_max);
    cout << "  SPH particles:     " << sysFSI.GetNumFluidMarkers() << endl;
    cout << "  Bndry BCE markers: " << sysFSI.GetNumBoundaryMarkers() << endl;
    cout << "  AABB:              " << aabb_min << "   " << aabb_max << endl;

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

        visFSI->SetTitle("Viper rover on SPH deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(init_loc + ChVector<>(0, 6, 0.5), init_loc);
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb_min.z(), aabb_max.z()));
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
