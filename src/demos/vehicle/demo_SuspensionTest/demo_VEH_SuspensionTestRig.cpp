// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================
double post_limit = 0.2;

// Simulation step size
double step_size = 1e-3;
double render_step_size = 1.0 / 50;  // Time interval between two render frames

// =============================================================================
// JSON file for suspension test rig
std::string suspensionTest_file("hmmwv/suspensionTest/HMMWV_ST_front.json");
// std::string suspensionTest_file("hmmwv/suspensionTest/HMMWV_ST_rear.json");

// JSON file for vehicle and axle index
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
int axle_index = 0;

// JSON files for tire models (rigid)
std::string rigidtire_file("hmmwv/tire/HMMWV_RigidTire.json");

// =============================================================================
int main(int argc, char* argv[]) {
    // Create and initialize the testing mechanism.
    ////ChSuspensionTestRig rig(vehicle::GetDataFile(suspensionTest_file));
    ChSuspensionTestRig rig(vehicle::GetDataFile(vehicle_file), axle_index);
    rig.Initialize(ChCoordsys<>());

    // Flat rigid terrain, height = 0.
    FlatTerrain flat_terrain(0);

    // Use rigid wheels to actuate suspension.
    auto tire_L = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
    auto tire_R = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
    tire_L->Initialize(rig.GetWheelBody(LEFT), LEFT);
    tire_R->Initialize(rig.GetWheelBody(RIGHT), RIGHT);

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(&rig, NULL, L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig.GetWheelPos(LEFT) + rig.GetWheelPos(RIGHT)), 2.0, 1.0);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver system and set the time response for keyboard inputs.
    ChIrrGuiDriverSTR driver(app, post_limit);
    double steering_time = 1.0;      // time to go from 0 to max
    double displacement_time = 2.0;  // time to go from 0 to max applied post motion
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetDisplacementDelta(render_step_size / displacement_time * post_limit);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TireForces tire_forces(2);
    WheelStates wheel_states(2);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }

        // Collect output data from modules
        double steering_input = driver.GetSteering();
        double left_input = driver.GetDisplacementLeft();
        double right_input = driver.GetDisplacementRight();

        tire_forces[0] = tire_L->GetTireForce();
        tire_forces[1] = tire_R->GetTireForce();
        wheel_states[0] = rig.GetWheelState(LEFT);
        wheel_states[1] = rig.GetWheelState(RIGHT);

        // Update modules (process inputs from other modules)
        double time = rig.GetChTime();
        driver.Synchronize(time);
        flat_terrain.Synchronize(time);
        tire_L->Synchronize(time, wheel_states[0], flat_terrain);
        tire_R->Synchronize(time, wheel_states[1], flat_terrain);
        rig.Synchronize(time, steering_input, left_input, right_input, tire_forces);
        app.Synchronize("", steering_input, 0, 0);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        flat_terrain.Advance(step);
        tire_L->Advance(step);
        tire_R->Advance(step);
        rig.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    return 0;
}
