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

//// RADU
//// Todo: why are the trailer tires not synchronized? fix!

#include <vector>

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

#include "chrono_thirdparty/filesystem/path.h"

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

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the tractor: specify if chassis is fixed, the suspension type
    // (SOLID_AXLE or MULTI_LINK) and the wheel visualization (PRIMITIVES or NONE)
    TT_Tractor vehicle(false, SuspensionType::MULTI_LINK, ChContactMethod::NSC);
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
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.5f, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<Generic_SimplePowertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    // Create the tires
    auto tire_FL = chrono_types::make_shared<Generic_RigidTire>("FL");
    auto tire_FR = chrono_types::make_shared<Generic_RigidTire>("FR");
    auto tire_RL = chrono_types::make_shared<Generic_RigidTire>("RL");
    auto tire_RR = chrono_types::make_shared<Generic_RigidTire>("RR");

    vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[0], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[1], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[0], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[1], VisualizationType::PRIMITIVES);

    // Create the trailer tires
    auto tr_tire_FL = chrono_types::make_shared<Generic_RigidTire>("FL");
    auto tr_tire_FR = chrono_types::make_shared<Generic_RigidTire>("FR");
    auto tr_tire_RL = chrono_types::make_shared<Generic_RigidTire>("RL");
    auto tr_tire_RR = chrono_types::make_shared<Generic_RigidTire>("RR");

    trailer.InitializeTire(tr_tire_FL, trailer.GetAxle(0)->m_wheels[0], VisualizationType::PRIMITIVES);
    trailer.InitializeTire(tr_tire_FR, trailer.GetAxle(0)->m_wheels[1], VisualizationType::PRIMITIVES);
    trailer.InitializeTire(tr_tire_RL, trailer.GetAxle(1)->m_wheels[0], VisualizationType::PRIMITIVES);
    trailer.InitializeTire(tr_tire_RR, trailer.GetAxle(1)->m_wheels[1], VisualizationType::PRIMITIVES);

    ChWheeledVehicleIrrApp app(&vehicle, L"Articulated Vehicle Demo");

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

    driver.Initialize();

// ---------------
// Simulation loop
// ---------------

#ifdef DEBUG_LOG
    GetLog() << "\n\n============ System Configuration ============\n";
    vehicle.LogHardpointLocations();
#endif

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

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

#ifdef DEBUG_LOG
        if (step_number % output_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            vehicle.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
        }
#endif

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        trailer.Synchronize(time, braking_input, terrain);
        vehicle.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        trailer.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
