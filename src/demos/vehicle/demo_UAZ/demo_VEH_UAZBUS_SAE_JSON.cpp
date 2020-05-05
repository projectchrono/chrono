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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Main driver function for a UAZ vehicle specified through JSON files.
// Example of leaf-spring Axles.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/ChConfigVehicle.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// JSON file for vehicle model
////std::string vehicle_file("uaz/vehicle/UAZBUS_Vehicle.json");
////std::string vehicle_file("uaz/vehicle/UAZ469_Vehicle.json");
std::string vehicle_file("uaz/vehicle/UAZBUS_SAEVehicle.json");

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// JSON file for powertrain (simple map)
std::string simple_map_powertrain_file("uaz/powertrain/UAZBUS_SimpleMapPowertrain.json");

// JSON files tire models
std::string tmeasy_front_tire_file("uaz/tire/UAZBUS_TMeasyTireFront.json");
std::string tmeasy_rear_tire_file("uaz/tire/UAZBUS_TMeasyTireRear.json");

// JSON files tire models (Pac02)
std::string pac02tire_file("uaz/tire/UAZBUS_Pac02Tire.json");

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Type of tire model (TMEASY, PAC02)
TireModelType tire_model = TireModelType::TMEASY;

// Initial vehicle position, avoid big fall heights with TMeasy
ChVector<> initLoc(0, 0, 0.38);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction

// Simulation step size
double step_size = 2e-3;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 50;

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 20.0;

// Output directories (Povray only)
const std::string out_dir = GetChronoOutputPath() + "UAZ_SAE_JSON";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<SimpleMapPowertrain>(vehicle::GetDataFile(simple_map_powertrain_file));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    switch (tire_model) {
        case TireModelType::TMEASY: {
            auto tireFL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_front_tire_file));
            auto tireFR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_front_tire_file));
            auto tireRL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_rear_tire_file));
            auto tireRR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_rear_tire_file));
            vehicle.InitializeTire(tireFL, vehicle.GetAxle(0)->m_wheels[0], VisualizationType::MESH);
            vehicle.InitializeTire(tireFR, vehicle.GetAxle(0)->m_wheels[1], VisualizationType::MESH);
            vehicle.InitializeTire(tireRL, vehicle.GetAxle(1)->m_wheels[0], VisualizationType::MESH);
            vehicle.InitializeTire(tireRR, vehicle.GetAxle(1)->m_wheels[1], VisualizationType::MESH);
        } break;

        case TireModelType::PAC02: {
            auto tireFL = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
            auto tireFR = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
            auto tireRL = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
            auto tireRR = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
            vehicle.InitializeTire(tireFL, vehicle.GetAxle(0)->m_wheels[0], VisualizationType::MESH);
            vehicle.InitializeTire(tireFR, vehicle.GetAxle(0)->m_wheels[1], VisualizationType::MESH);
            vehicle.InitializeTire(tireRL, vehicle.GetAxle(1)->m_wheels[0], VisualizationType::MESH);
            vehicle.InitializeTire(tireRR, vehicle.GetAxle(1)->m_wheels[1], VisualizationType::MESH);
        } break;

        default:
            GetLog() << "Unsupported tire model selected!\n";
            return 1;
    }

    // Create the Irrlicht visualization
    ChWheeledVehicleIrrApp app(&vehicle, L"UAZ (JSON) Vehicle Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create an interactive driver
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // Set file with driver input time series
    driver.SetInputDataFile(vehicle::GetDataFile(driver_file));

    driver.Initialize();

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
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counters
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;
    int render_frame = 0;

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        if (povray_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(vehicle.GetSystem(), filename);
            render_frame++;
        }

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
