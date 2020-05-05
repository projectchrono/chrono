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
// Main driver function for a vehicle specified through JSON files.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/Pac02Tire.h"

#include "chrono_vehicle/ChConfigVehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
// specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
////std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");
////std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_4WD.json");
////std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones.json");
////std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones_ARB.json");
////std::string vehicle_file("generic/vehicle/Vehicle_MultiLinks.json");
////std::string vehicle_file("generic/vehicle/Vehicle_SolidAxles.json");
////std::string vehicle_file("generic/vehicle/Vehicle_ThreeAxles.json");
////std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Independent.json");
////std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Shared.json");
////std::string vehicle_file("generic/vehicle/Vehicle_MacPhersonStruts.json");
////std::string vehicle_file("generic/vehicle/Vehicle_SemiTrailingArm.json");
////std::string vehicle_file("generic/vehicle/Vehicle_ThreeLinkIRS.json");

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// JSON file for powertrain (simple)
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// JSON files tire models (rigid)
std::string rigidtire_file("hmmwv/tire/HMMWV_RigidTire.json");
////std::string rigidtire_file("hmmwv/tire/HMMWV_RigidMeshTire.json");
////std::string rigidtire_file("generic/tire/RigidTire.json");

// JSON files tire models (Fiala)
std::string fialatire_file("hmmwv/tire/HMMWV_Fiala_converted.json");

// JSON files tire models (TMeasy)
std::string tmeasytire_file("hmmwv/tire/HMMWV_TMeasy_converted.json");

// JSON files tire models (Pac89)
std::string pac89tire_file("hmmwv/tire/HMMWV_Pac89Tire.json");

std::string pac02tire_file("hmmwv/tire/HMMWV_Pac02Tire.json");

// Type of tire model (RIGID, FIALA, TMEASY, PAC89)
TireModelType tire_model = TireModelType::PAC02;

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.6);

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

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 20.0;

// Output directories (Povray only)
const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";
const std::string pov_dir = out_dir + "/POVRAY";

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
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<SimplePowertrain>(vehicle::GetDataFile(simplepowertrain_file));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        switch (tire_model) {
            default:
            case TireModelType::RIGID: {
                auto tireL = chrono_types::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
                auto tireR = chrono_types::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH);
                break;
            }
            case TireModelType::TMEASY: {
                auto tireL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasytire_file));
                auto tireR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasytire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH);
                break;
            }
            case TireModelType::FIALA: {
                auto tireL = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
                auto tireR = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH);
                break;
            }
            case TireModelType::PAC89: {
                auto tireL = chrono_types::make_shared<Pac89Tire>(vehicle::GetDataFile(pac89tire_file));
                auto tireR = chrono_types::make_shared<Pac89Tire>(vehicle::GetDataFile(pac89tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH);
                break;
            }
            case TireModelType::PAC02: {
                auto tireL = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
                auto tireR = chrono_types::make_shared<Pac02Tire>(vehicle::GetDataFile(pac02tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH);
                break;
            }
        }
    }

#ifdef USE_IRRLICHT

    ChWheeledVehicleIrrApp app(&vehicle, L"Vehicle Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    /*
    bool do_shadows = false; // shadow map is experimental
    irr::scene::ILightSceneNode* mlight = 0;

    if (do_shadows) {
      mlight = application.AddLightWithShadow(
        irr::core::vector3df(10.f, 30.f, 60.f),
        irr::core::vector3df(0.f, 0.f, 0.f),
        150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
    } else {
      application.AddTypicalLights(
        irr::core::vector3df(30.f, -30.f, 100.f),
        irr::core::vector3df(30.f, 50.f, 100.f),
        250, 130);
    }

    if (do_shadows)
        application.AddShadowAll();
    */

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

#else

    ChDataDriver driver(vehicle, vehicle::GetDataFile(driver_file));

#endif

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ---------------
    // Simulation loop
    // ---------------

#ifdef USE_IRRLICHT

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Update the position of the shadow mapping so that it follows the car
        ////if (do_shadows) {
        ////  ChVector<> lightaim = vehicle.GetChassisPos();
        ////  ChVector<> lightpos = vehicle.GetChassisPos() + ChVector<>(10, 30, 60);
        ////  irr::core::vector3df mlightpos((irr::f32)lightpos.x, (irr::f32)lightpos.y, (irr::f32)lightpos.z);
        ////  irr::core::vector3df mlightaim((irr::f32)lightaim.x, (irr::f32)lightaim.y, (irr::f32)lightaim.z);
        ////  application.GetEffects()->getShadowLight(0).setPosition(mlightpos);
        ////  application.GetEffects()->getShadowLight(0).setTarget(mlightaim);
        ////  mlight->setPosition(mlightpos);
        ////}

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

#else

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    double time = 0;
    int step_number = 0;
    int render_frame = 0;
    char filename[100];

    while (time < tend) {
        if (step_number % render_steps == 0) {
            // Output render data
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(vehicle.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << std::endl;
            std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
                      << "   braking:  " << driver.GetBraking() << std::endl;
            std::cout << std::endl;
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        // Increment frame number
        step_number++;
    }

#endif

    return 0;
}
