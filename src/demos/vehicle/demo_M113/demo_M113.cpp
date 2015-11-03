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
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#include "m113/M113_Vehicle.h"
#include "m113/M113_SimplePowertrain.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace m113;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(0, 0, 2.0);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(-2.0, 0.0, 0.5);

// =============================================================================

// Simple powertrain model
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// =============================================================================
int main(int argc, char* argv[]) {
    // Create the M113 vehicle.
    M113_Vehicle vehicle(true, PRIMITIVES, PRIMITIVES, ChMaterialSurfaceBase::DEM);

    // vehicle.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));
    vehicle.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    vehicle.GetSystem()->SetIterLCPmaxItersSpeed(150);
    vehicle.GetSystem()->SetIterLCPmaxItersStab(150);
    vehicle.GetSystem()->SetTol(0);
    vehicle.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    vehicle.GetSystem()->SetMinBounceSpeed(2.0);
    vehicle.GetSystem()->SetIterLCPomega(0.8);
    vehicle.GetSystem()->SetIterLCPsharpnessLambda(1.0);

    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem());
    terrain.SetContactMaterial(0.9f, 0.01f, 2e7f, 0.3f);
    terrain.SetColor(ChColor(0.5f, 0.8f, 0.5f));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // Create and initialize the powertrain system.
    M113_SimplePowertrain powertrain;
    powertrain.Initialize();

    // Create the vehicle Irrlicht application.
    ChTrackedVehicleIrrApp app(&vehicle, &powertrain, L"M113 Vehicle Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 3.0, 1.0);
    ////app.SetChaseCameraPosition(trackPoint + ChVector<>(0, 3, 0));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver system and set the time response for keyboard inputs.
    ChIrrGuiDriver driver(app);
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TrackShoeForces shoe_forces_left(vehicle.GetTrackAssembly(LEFT)->GetNumTrackShoes());
    TrackShoeForces shoe_forces_right(vehicle.GetTrackAssembly(RIGHT)->GetNumTrackShoes());

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
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();
        double powertrain_torque = powertrain.GetOutputTorque();
        double driveshaft_speed = vehicle.GetDriveshaftSpeed();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver.Update(time);
        terrain.Update(time);
        powertrain.Update(time, throttle_input, driveshaft_speed);
        vehicle.Update(time, steering_input, braking_input, powertrain_torque, shoe_forces_left, shoe_forces_right);
        app.Update("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    return 0;
}
