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
// Authors: Harry Zhang, Radu Serban
// =============================================================================
//
// Demonstration of using rigid terrain and SCM terrain patches in the same
// environment.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <iomanip>

#include "chrono/utils/ChOpenMP.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Patch types (RIGID or SCM)
enum class PatchType { RIGID, SCM };
PatchType patches[2] = {PatchType::SCM, PatchType::RIGID};

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

// Frictional contact formulation.
// Smooth contact (SMC) leads to stiffer problems, potentially requiring a smaller step size,
// but the cost per time step of contact calculation is low.
// Non-smooth contact (NSC) can use larger step sizes than SCM but has high cost per step.
ChContactMethod contact_method = ChContactMethod::SMC;

// Integration step size.
double step_size = 2e-3;

// Set number of OpenMP threads (settings for optimum performance are hardware-specific).
// Here,
//   num_threads_eigen     - affects SCM calculations
//   num_threads_collision - affects Bullet collision detection
// See documentation for ChSystem::SetNumThreads for more details.
int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
int num_threads_collision = std::min(8, ChOMP::GetNumProcs());
int num_threads_eigen = 1;

// High resolution height-map images (false: 64x64, true: 200x200).
// A high resolution of the height-map images results in dense meshes for rigid terrain patches.
// This can significantly impact performance of the rigid-rigid collision detection algorithm.
bool high_res = false;

// Visualization output
bool render = true;
double render_fps = 120;
bool img_output = false;

// SCM terrain visualization options.
bool render_wireframe = true;  // render wireframe (flat otherwise)
bool apply_texture = false;    // add texture
bool render_sinkage = true;    // use false coloring for sinkage visualization

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // ------------------------
    // Create the HMMWV vehicle
    // ------------------------

    HMMWV_Full hmmwv;
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-5, 0, 0.5), QUNIT));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // Extract underlying Chrono system
    auto sys = hmmwv.GetSystem();

    // Enable the Bullet collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set number of threads used for different Chrono algorithms
    sys->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    // ----------------------
    // Create terrain patches
    // ----------------------

    // Height-map image files
    auto hm_concave = high_res ? vehicle::GetDataFile("terrain/height_maps/concave200.bmp")
                               : vehicle::GetDataFile("terrain/height_maps/concave64.bmp");
    auto hm_convex = high_res ? vehicle::GetDataFile("terrain/height_maps/convex200.bmp")
                              : vehicle::GetDataFile("terrain/height_maps/convex64.bmp");

    // Create rigid terrain contact material consistent with the current contact formulation
    ChContactMaterialData mat_data;
    mat_data.mu = 0.9f;
    mat_data.cr = 0.01f;
    mat_data.Y = 2e7f;
    auto mat = mat_data.CreateMaterial(contact_method);

    // Create rigid terrain patches
    RigidTerrain rigid_terrain(sys);
    {
        // x: [-20 ... 0]
        auto patch = rigid_terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(-10, 0, 0), QUNIT), 20, 10);
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 10, 5);
    }
    if (patches[0] == PatchType::RIGID) {
        // x: [0 ... 10]
        auto patch0 = rigid_terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(5, 0, -1), QUNIT), hm_concave, 10, 10, 0, 1);
        patch0->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 5, 5);
    }
    if (patches[1] == PatchType::RIGID) {
        // x: [10 ... 20]
        auto patch1 = rigid_terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(15, 0, 0), QUNIT), hm_convex, 10, 10, 0, 1);
        patch1->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 5, 5);
    }
    {
        // x: [20 ... 40]
        auto patch = rigid_terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(30, 0, 0), QUNIT), 20, 10);
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 10, 5);
    }

    rigid_terrain.Initialize();

    // Create SCM terrain patches
    // Notes:
    // - make sure to define a bounding AABB for each SCM terrain patch;
    //   this prevent the SCM grid from extending beyond those limits and interfering with the rigid terrain patches.
    // - define active domains for bodies interacting with the SCM terrain patches;
    //   this minimizes the number of ray casts; for best performance, define an active domain for each tire.

    ChAABB scm_patch_aabb(ChVector3d(-5, -5, -2), ChVector3d(5, 5, 2));  // AABB for an SCM patch
    ChVector3d hmmwv_tire_aabb_size(1.0, 0.6, 1.0);                      // dimensions of OOBB associated with a tire

    if (patches[0] == PatchType::SCM) {
        // x: [0 ... 10]
        SCMTerrain scm0(sys);
        scm0.SetReferenceFrame(ChCoordsys<>(ChVector3d(5, 0, 0), QUNIT));
        scm0.SetBoundary(scm_patch_aabb);
        for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
            scm0.AddActiveDomain(axle->m_wheels[0]->GetSpindle(), VNULL, hmmwv_tire_aabb_size);
            scm0.AddActiveDomain(axle->m_wheels[1]->GetSpindle(), VNULL, hmmwv_tire_aabb_size);
        }
        scm0.Initialize(hm_concave, 10, 10, -1, 0, 0.05);

        scm0.GetMesh()->SetWireframe(render_wireframe);
        if (apply_texture)
            scm0.GetMesh()->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 10, 10);
        if (render_sinkage)
            scm0.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    }
    if (patches[1] == PatchType::SCM) {
        // x: [10 ... 20]
        SCMTerrain scm1(sys);
        scm1.SetReferenceFrame(ChCoordsys<>(ChVector3d(15, 0, 0), QUNIT));
        scm1.SetBoundary(scm_patch_aabb);
        for (auto& axle : hmmwv.GetVehicle().GetAxles()) {
            scm1.AddActiveDomain(axle->m_wheels[0]->GetSpindle(), VNULL, hmmwv_tire_aabb_size);
            scm1.AddActiveDomain(axle->m_wheels[1]->GetSpindle(), VNULL, hmmwv_tire_aabb_size);
        }
        scm1.Initialize(hm_convex, 10, 10, 0, 1, 0.05);

        scm1.GetMesh()->SetWireframe(render_wireframe);
        if (apply_texture)
            scm1.GetMesh()->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 10, 10);
        if (render_sinkage)
            scm1.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    }

    // ----------------------------
    // Create an interactive driver
    // ----------------------------

    ChInteractiveDriver driver(hmmwv.GetVehicle());
    driver.SetSteeringDelta(1.0 / 50);
    driver.SetThrottleDelta(1.0 / 50);
    driver.SetBrakingDelta(3.0 / 50);
    driver.Initialize();

    // -----------------------------
    // Create run-time visualization
    // -----------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                // Create the vehicle Irrlicht interface
                auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
                vis_irr->SetWindowTitle("Mixed Terrain Demo");
                vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.75);
                vis_irr->Initialize();
                vis_irr->AddLightDirectional();
                vis_irr->AddSkyBox();
                vis_irr->AddLogo();
                vis_irr->AttachVehicle(&hmmwv.GetVehicle());
                vis_irr->AttachDriver(&driver);

                vis = vis_irr;
#endif
                break;
            }

            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                // Create the vehicle VSG interface
                auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
                vis_vsg->SetWindowTitle("Mixed Terrain Demo");
                vis_vsg->SetWindowSize(1280, 800);
                vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 8.0, 0.75);
                vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
                vis_vsg->AttachDriver(&driver);
                vis_vsg->AttachTerrain(&rigid_terrain);
                vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
                vis_vsg->EnableShadows();
                vis_vsg->EnableSkyBox();
                vis_vsg->Initialize();

                vis = vis_vsg;
#endif
                break;
            }
        }
    }

    // -------------------------
    // Create output directories
    // -------------------------

    const std::string out_dir = GetChronoOutputPath() + "DEMO_RIGID_SCM_PATCHES";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/img"))) {
            std::cout << "Error creating directory " << out_dir + "/img" << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    int render_frame = 0;
    double time = 0;
    double stop_time = 20;

    while (time < stop_time) {
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;

            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (img_output) {
                std::ostringstream filename;
                filename << out_dir << "/img/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        DriverInputs inputs = driver.GetInputs();

        driver.Synchronize(time);
        rigid_terrain.Synchronize(time);
        hmmwv.Synchronize(time, inputs, rigid_terrain);
        vis->Synchronize(time, inputs);

        driver.Advance(step_size);
        rigid_terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        time += step_size;
    }

    return 0;
}
