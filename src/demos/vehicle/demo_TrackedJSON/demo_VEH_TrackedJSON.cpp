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
// Main driver function for a tracked vehicle specified through JSON files.
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
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"

#include "chrono_thirdparty/filesystem/path.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

// ...and specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// JSON file for vehicle model
std::string vehicle_file("M113/vehicle/M113_Vehicle_SinglePin.json");
////std::string vehicle_file("M113/vehicle/M113_Vehicle_SinglePin_box.json");
////std::string vehicle_file("M113/vehicle/M113_Vehicle_SinglePin_mesh.json");
////std::string vehicle_file("M113/vehicle/M113_Vehicle_DoublePin.json");

// JSON file for powertrain
std::string simplepowertrain_file("M113/powertrain/M113_SimplePowertrain.json");

// JSON files for terrain (rigid plane)
std::string rigidterrain_file("terrain/RigidPlane.json");

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Simulation step size
double step_size = 4e-3;

// Simulation length (Povray only)
double tend = 10.0;
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories (Povray only)
const std::string out_dir = GetChronoOutputPath() + "M113_JSON";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Contact formulation
    ChContactMethod contact_method = ChContactMethod::NSC;

    //// NOTE
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.  
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    ////
    //// For now, use ChContactMethod::NSC for a double-pin track model

    // Create the vehicle system
    TrackedVehicle vehicle(vehicle::GetDataFile(vehicle_file), contact_method);

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // Initialize the vehicle at the specified position.
    vehicle.Initialize(ChCoordsys<>(ChVector<>(0, 0, 1.2), QUNIT));

    // Set visualization type for vehicle components
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelAssemblyVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Solver settings.
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(50);
    solver->SetOmega(0.8);
    solver->SetSharpnessLambda(1.0);
    vehicle.GetSystem()->SetSolver(solver);

    vehicle.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    vehicle.GetSystem()->SetMinBounceSpeed(2.0);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<SimplePowertrain>(vehicle::GetDataFile(simplepowertrain_file));
    vehicle.InitializePowertrain(powertrain);

#ifdef USE_IRRLICHT

    ChTrackedVehicleIrrApp app(&vehicle, L"JSON Tracked Vehicle Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 0.0), 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
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

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time

#ifdef USE_IRRLICHT

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
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

    double time = 0;
    int step_number = 0;
    int render_frame = 0;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

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
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
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
