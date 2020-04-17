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
// Main function for an articulated vehicle (steering applied to the joint
// connecting the front and rear sides).
//
// Driver inputs are obtained from the keyboard (interactive driver model).
//
// The front_side reference frame has Z up, X towards the front of the vehicle,
// and Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_RigidMeshTire.h"
#include "chrono_models/vehicle/generic/Generic_FialaTire.h"

#include "subsystems/Articulated_Front.h"
#include "subsystems/Articulated_Rear.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// =============================================================================

// Initial front_side position
ChVector<> initLoc(0, 0, 0.5);

// Initial front_side orientation
ChQuaternion<> initRot = Q_from_AngZ(CH_C_PI / 10);

// Type of tire model (RIGID, RIGID_MESH, or FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 0.001;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the front side
    Articulated_Front front_side(false, ChContactMethod::NSC);
    front_side.Initialize(ChCoordsys<>(initLoc, initRot));
    front_side.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    front_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    front_side.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    front_side.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the rear side
    Articulated_Rear rear_side(std::static_pointer_cast<Articulated_Chassis>(front_side.GetChassis()));
    rear_side.Initialize();
    rear_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    rear_side.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the terrain
    RigidTerrain terrain(front_side.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch =
        terrain.AddPatch(patch_mat, ChVector<>(0, 0, terrainHeight), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.5f, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<Generic_SimplePowertrain>("Powertrain");
    front_side.InitializePowertrain(powertrain);

    // Create and initialize the front and rear tires
    switch (tire_model) {
        case TireModelType::RIGID: {
            auto tire_FL = chrono_types::make_shared<Generic_RigidTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_RigidTire>("FR");
            front_side.InitializeTire(tire_FL, front_side.GetAxle(0)->m_wheels[0], VisualizationType::MESH);
            front_side.InitializeTire(tire_FR, front_side.GetAxle(0)->m_wheels[1], VisualizationType::MESH);

            auto tire_RL = chrono_types::make_shared<Generic_RigidTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_RigidTire>("RR");
            rear_side.InitializeTire(tire_RL, rear_side.GetAxle()->m_wheels[0], VisualizationType::MESH);
            rear_side.InitializeTire(tire_RR, rear_side.GetAxle()->m_wheels[1], VisualizationType::MESH);

            break;
        }
        case TireModelType::RIGID_MESH: {
            auto tire_FL = chrono_types::make_shared<Generic_RigidMeshTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_RigidMeshTire>("FR");
            front_side.InitializeTire(tire_FL, front_side.GetAxle(0)->m_wheels[0], VisualizationType::MESH);
            front_side.InitializeTire(tire_FR, front_side.GetAxle(0)->m_wheels[1], VisualizationType::MESH);
 
            auto tire_RL = chrono_types::make_shared<Generic_RigidMeshTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_RigidMeshTire>("RR");
            rear_side.InitializeTire(tire_RL, rear_side.GetAxle()->m_wheels[0], VisualizationType::MESH);
            rear_side.InitializeTire(tire_RR, rear_side.GetAxle()->m_wheels[1], VisualizationType::MESH);

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<Generic_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_FialaTire>("FR");
            front_side.InitializeTire(tire_FL, front_side.GetAxle(0)->m_wheels[0], VisualizationType::MESH);
            front_side.InitializeTire(tire_FR, front_side.GetAxle(0)->m_wheels[1], VisualizationType::MESH);
 
            auto tire_RL = chrono_types::make_shared<Generic_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_FialaTire>("RR");
            rear_side.InitializeTire(tire_RL, rear_side.GetAxle()->m_wheels[0], VisualizationType::MESH);
            rear_side.InitializeTire(tire_RR, rear_side.GetAxle()->m_wheels[1], VisualizationType::MESH);

            break;
        }
        default:
            std::cout << "Tire type not supported!" << std::endl;
            return 1;
    }

    // Initialize Irrlicht app
    ChWheeledVehicleIrrApp app(&front_side, L"Articulated Vehicle Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize interactive driver
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

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        double time = front_side.GetSystem()->GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        rear_side.Synchronize(time, steering_input, braking_input, terrain);
        front_side.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        rear_side.Advance(step_size);
        front_side.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
