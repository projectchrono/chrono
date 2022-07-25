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
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for the Gator full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, RIGID_MESH, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = step_size;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Gator";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create vehicle
    // --------------

    Gator gator;
    gator.SetContactMethod(contact_method);
    gator.SetChassisCollisionType(chassis_collision_type);
    gator.SetChassisFixed(false);
    gator.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    gator.SetTireType(tire_model);
    gator.SetTireStepSize(tire_step_size);
    gator.SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator.Initialize();

    gator.SetChassisVisualizationType(chassis_vis_type);
    gator.SetSuspensionVisualizationType(suspension_vis_type);
    gator.SetSteeringVisualizationType(steering_vis_type);
    gator.SetWheelVisualizationType(wheel_vis_type);
    gator.SetTireVisualizationType(tire_vis_type);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(gator.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(10, 0, 0), Q_from_AngY(-10 * CH_C_DEG_TO_RAD)),
                                     5, 10);
            patch->SetColor(ChColor(0.6f, 0.5f, 0.2f));
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }

    terrain.Initialize();

    ////auto truss_mesh = chrono_types::make_shared<ChObjFileShape>();
    ////truss_mesh->SetFilename(GetChronoDataFile("vehicle/gator/gator_chassis.obj"));
    ////patch->GetGroundBody()->AddVisualShape(truss_mesh, ChFrame<>(ChVector<>(-10, -2, 3)));

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // -------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Gator Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0.05);
    vis->Initialize();
    vis->AddLightDirectional(70, 20);
    ////vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // output vehicle mass
    gator.GetVehicle().LogSubsystemTypes();
    std::cout << "\nVehicle mass: " << gator.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    gator.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = gator.GetSystem()->GetChTime();

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(gator.GetSystem(), filename);
            }

            render_frame++;
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
