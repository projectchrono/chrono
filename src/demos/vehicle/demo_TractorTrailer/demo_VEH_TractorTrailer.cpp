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
// Authors: Alessandro Tasora, Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Main driver function for an articulated vehicle, using rigid tire-terrain contact.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_FuncDriver.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "subsystems/TT_Tractor.h"
#include "subsystems/TT_Trailer.h"

// Comment the following line to disable Irrlicht visualization
#define USE_IRRLICHT

// DEBUGGING:  Uncomment the following line to print shock data
//#define DEBUG_LOG

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// =============================================================================

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.0);

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
double step_size = 0.001;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 1;  // once a second

#ifdef USE_IRRLICHT
// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);
#else
double tend = 20.0;

const std::string out_dir = GetChronoOutputPath() + "TRACTOR_TRAILER";
const std::string pov_dir = out_dir + "/POVRAY";
#endif

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the tractor: specify if chassis is fixed, the suspension type
    // (SOLID_AXLE or MULTI_LINK) and the wheel visualization (PRIMITIVES or NONE)
    TT_Tractor vehicle(false, SuspensionType::MULTI_LINK);
    vehicle.Initialize(ChCoordsys<>(initLoc + ChVector<>(0, 0, 0), initRot));
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the trailer: specify if chassis is fixed, the suspension type
    // (SOLID_AXLE or MULTI_LINK) and the wheel visualization (PRIMITIVES or NONE)
    TT_Trailer trailer(vehicle.GetSystem(), false, SuspensionType::MULTI_LINK);
    trailer.Initialize(ChCoordsys<>(initLoc + ChVector<>(-6, 0, 0), initRot), true, vehicle.GetChassisBody());
    trailer.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem());
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(0.5f, 0.5f, 1));
    terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // Create and initialize the powertrain system
    Generic_SimplePowertrain powertrain;

    powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft());

    // Create the tires
    Generic_RigidTire tire_front_left("FL");
    Generic_RigidTire tire_front_right("FR");
    Generic_RigidTire tire_rear_left("RL");
    Generic_RigidTire tire_rear_right("RR");

    tire_front_left.Initialize(vehicle.GetWheelBody(FRONT_LEFT), LEFT);
    tire_front_right.Initialize(vehicle.GetWheelBody(FRONT_RIGHT), RIGHT);
    tire_rear_left.Initialize(vehicle.GetWheelBody(REAR_LEFT), LEFT);
    tire_rear_right.Initialize(vehicle.GetWheelBody(REAR_RIGHT), RIGHT);

    tire_front_left.SetVisualizationType(VisualizationType::PRIMITIVES);
    tire_front_right.SetVisualizationType(VisualizationType::PRIMITIVES);
    tire_rear_left.SetVisualizationType(VisualizationType::PRIMITIVES);
    tire_rear_right.SetVisualizationType(VisualizationType::PRIMITIVES);

    // Create the trailer tires
    Generic_RigidTire tr_tire_front_left("FL");
    Generic_RigidTire tr_tire_front_right("FR");
    Generic_RigidTire tr_tire_rear_left("RL");
    Generic_RigidTire tr_tire_rear_right("RR");

    tr_tire_front_left.Initialize(trailer.GetWheelBody(FRONT_LEFT), LEFT);
    tr_tire_front_right.Initialize(trailer.GetWheelBody(FRONT_RIGHT), RIGHT);
    tr_tire_rear_left.Initialize(trailer.GetWheelBody(REAR_LEFT), LEFT);
    tr_tire_rear_right.Initialize(trailer.GetWheelBody(REAR_RIGHT), RIGHT);

    tr_tire_front_left.SetVisualizationType(VisualizationType::PRIMITIVES);
    tr_tire_front_right.SetVisualizationType(VisualizationType::PRIMITIVES);
    tr_tire_rear_left.SetVisualizationType(VisualizationType::PRIMITIVES);
    tr_tire_rear_right.SetVisualizationType(VisualizationType::PRIMITIVES);

#ifdef USE_IRRLICHT
    ChWheeledVehicleIrrApp app(&vehicle, &powertrain, L"Articulated Vehicle Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

#else
    Generic_FuncDriver driver(vehicle);
#endif

    driver.Initialize();

// ---------------
// Simulation loop
// ---------------

#ifdef DEBUG_LOG
    GetLog() << "\n\n============ System Configuration ============\n";
    vehicle.LogHardpointLocations();
#endif

    // Inter-module communication data
    TerrainForces tire_forces(4);
    TerrainForces tr_tire_forces(4);
    WheelState wheel_states[4];
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;

#ifdef USE_IRRLICHT

    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }

#ifdef DEBUG_LOG
        if (step_number % output_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            vehicle.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
        }
#endif

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();

        powertrain_torque = powertrain.GetOutputTorque();

        tire_forces[FRONT_LEFT.id()] = tire_front_left.GetTireForce();
        tire_forces[FRONT_RIGHT.id()] = tire_front_right.GetTireForce();
        tire_forces[REAR_LEFT.id()] = tire_rear_left.GetTireForce();
        tire_forces[REAR_RIGHT.id()] = tire_rear_right.GetTireForce();

        tr_tire_forces[FRONT_LEFT.id()] = tr_tire_front_left.GetTireForce();
        tr_tire_forces[FRONT_RIGHT.id()] = tr_tire_front_right.GetTireForce();
        tr_tire_forces[REAR_LEFT.id()] = tr_tire_rear_left.GetTireForce();
        tr_tire_forces[REAR_RIGHT.id()] = tr_tire_rear_right.GetTireForce();

        driveshaft_speed = vehicle.GetDriveshaftSpeed();

        wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
        wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);
        wheel_states[REAR_LEFT.id()] = vehicle.GetWheelState(REAR_LEFT);
        wheel_states[REAR_RIGHT.id()] = vehicle.GetWheelState(REAR_RIGHT);

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        driver.Synchronize(time);

        terrain.Synchronize(time);

        tire_front_left.Synchronize(time, wheel_states[FRONT_LEFT.id()], terrain);
        tire_front_right.Synchronize(time, wheel_states[FRONT_RIGHT.id()], terrain);
        tire_rear_left.Synchronize(time, wheel_states[REAR_LEFT.id()], terrain);
        tire_rear_right.Synchronize(time, wheel_states[REAR_RIGHT.id()], terrain);

        powertrain.Synchronize(time, throttle_input, driveshaft_speed);

        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        trailer.Synchronize(time, braking_input, tr_tire_forces);

        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);

        driver.Advance(step);

        terrain.Advance(step);

        tire_front_right.Advance(step);
        tire_front_left.Advance(step);
        tire_rear_right.Advance(step);
        tire_rear_left.Advance(step);

        powertrain.Advance(step);

        vehicle.Advance(step);

        app.Advance(step);

        // Increment frame number
        step_number++;
    }

#else

    int render_frame = 0;

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
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
            std::cout << "             throttle: " << driver.GetThrottle() << " steering: " << driver.GetSteering()
                      << std::endl;
            std::cout << std::endl;
            render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();

        powertrain_torque = powertrain.GetOutputTorque();

        tire_forces[FRONT_LEFT.id()] = tire_front_left.GetTireForce();
        tire_forces[FRONT_RIGHT.id()] = tire_front_right.GetTireForce();
        tire_forces[REAR_LEFT.id()] = tire_rear_left.GetTireForce();
        tire_forces[REAR_RIGHT.id()] = tire_rear_right.GetTireForce();

        driveshaft_speed = vehicle.GetDriveshaftSpeed();

        wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
        wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);
        wheel_states[REAR_LEFT.id()] = vehicle.GetWheelState(REAR_LEFT);
        wheel_states[REAR_RIGHT.id()] = vehicle.GetWheelState(REAR_RIGHT);

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        driver.Synchronize(time);

        terrain.Synchronize(time);

        tire_front_left.Synchronize(time, wheel_states[FRONT_LEFT.id()], terrain);
        tire_front_right.Synchronize(time, wheel_states[FRONT_RIGHT.id()], terrain);
        tire_rear_left.Synchronize(time, wheel_states[REAR_LEFT.id()], terrain);
        tire_rear_right.Synchronize(time, wheel_states[REAR_RIGHT.id()], terrain);

        powertrain.Synchronize(time, throttle_input, driveshaft_speed);

        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);

        terrain.Advance(step_size);

        tire_front_right.Advance(step_size);
        tire_front_left.Advance(step_size);
        tire_rear_right.Advance(step_size);
        tire_rear_left.Advance(step_size);

        powertrain.Advance(step_size);

        vehicle.Advance(step_size);

        // Increment frame number
        step_number++;
    }

#endif

    return 0;
}
