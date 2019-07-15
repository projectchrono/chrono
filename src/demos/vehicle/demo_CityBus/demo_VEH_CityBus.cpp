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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// Main driver function for the City Bus full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include <fstream>
#include <iostream>
#include "chrono_models/vehicle/citybus/CityBus.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::citybus;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE)
// PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD)
// DrivelineType drive_type = DrivelineType::FWD;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::Type terrain_model = RigidTerrain::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 3e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "CityBus";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the City Bus vehicle, set parameters, and initialize
    CityBus my_bus;
    my_bus.SetContactMethod(contact_method);
    my_bus.SetChassisCollisionType(chassis_collision_type);
    my_bus.SetChassisFixed(false);
    my_bus.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    // my_bus.SetPowertrainType(powertrain_model);
    // my_bus.SetDriveType(drive_type);
    my_bus.SetTireType(tire_model);
    my_bus.SetTireStepSize(tire_step_size);
    my_bus.SetVehicleStepSize(step_size);
    my_bus.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;  // : VisualizationType::PRIMITIVES;

    my_bus.SetChassisVisualizationType(chassis_vis_type);
    my_bus.SetSuspensionVisualizationType(suspension_vis_type);
    my_bus.SetSteeringVisualizationType(steering_vis_type);
    my_bus.SetWheelVisualizationType(wheel_vis_type);
    my_bus.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_bus.GetSystem());

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::BOX:
            patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                     ChVector<>(terrainLength, terrainWidth, 10));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::HEIGHT_MAP:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::MESH:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_bus.GetVehicle(), &my_bus.GetPowertrain(), L"City Bus Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_bus.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_bus.GetVehicle().GetVehicleMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    if (contact_vis) {
        app.SetSymbolscale(1e-4);
        app.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    }

    /*using namespace std;
    ofstream myfile;
    myfile.open("DEMO_OUTPUT/CityBus/exampleRight.txt");

    int count = 0;
    */

    while (app.GetDevice()->run()) {
        time = my_bus.GetSystem()->GetChTime();
        /*count++;
        if (count % 50 == 0) {


            myfile << "VEHICLE pos: " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(0.0, 1.25, 0.0)).x();
            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(0.0, 1.25, 0.0)).y();

            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(0.0, -1.25, 0.0)).x();
            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(0.0, -1.25, 0.0)).y();

            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(-7.184, 1.25,0.0)).x();
            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(-7.184, 1.25, 0.0)).y();

            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(-7.184, -1.25, 0.0)).x();
            myfile << " " << my_bus.GetVehicle().GetVehiclePointLocation(ChVector<>(-7.184, -1.25, 0.0)).y()<<
        std::endl;
        }
        if (time > 50)
            myfile.close();
        */

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_bus.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_bus.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << steering_input << throttle_input << braking_input << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_bus.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        my_bus.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}
