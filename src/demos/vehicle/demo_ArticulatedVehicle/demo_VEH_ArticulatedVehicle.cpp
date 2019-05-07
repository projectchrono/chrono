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
    Articulated_Front front_side(false);
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
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.5f, 0.5f, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the powertrain system
    Generic_SimplePowertrain powertrain("Powertrain");
    powertrain.Initialize(front_side.GetChassisBody(), front_side.GetDriveshaft());

    // Create the front tires
    std::unique_ptr<ChTire> tire_FL;
    std::unique_ptr<ChTire> tire_FR;
    switch (tire_model) {
        case TireModelType::RIGID:
            tire_FL = std::unique_ptr<ChTire>(new Generic_RigidTire("FL"));
            tire_FR = std::unique_ptr<ChTire>(new Generic_RigidTire("FR"));
            break;
        case TireModelType::RIGID_MESH:
            tire_FL = std::unique_ptr<ChTire>(new Generic_RigidMeshTire("FL"));
            tire_FR = std::unique_ptr<ChTire>(new Generic_RigidMeshTire("FR"));
            break;
        case TireModelType::FIALA:
            tire_FL = std::unique_ptr<ChTire>(new Generic_FialaTire("FL"));
            tire_FR = std::unique_ptr<ChTire>(new Generic_FialaTire("FR"));
            break;
        default:
            std::cout << "Tire type not supported!" << std::endl;
            return 1;
    }

    tire_FL->Initialize(front_side.GetWheelBody(FRONT_LEFT), LEFT);
    tire_FR->Initialize(front_side.GetWheelBody(FRONT_RIGHT), RIGHT);
    tire_FL->SetVisualizationType(VisualizationType::MESH);
    tire_FR->SetVisualizationType(VisualizationType::MESH);

    // Create the rear tires
    std::unique_ptr<ChTire> tire_RL;
    std::unique_ptr<ChTire> tire_RR;
    switch (tire_model) {
        case TireModelType::RIGID:
            tire_RL = std::unique_ptr<ChTire>(new Generic_RigidTire("RL"));
            tire_RR = std::unique_ptr<ChTire>(new Generic_RigidTire("RR"));
            break;
        case TireModelType::RIGID_MESH:
            tire_RL = std::unique_ptr<ChTire>(new Generic_RigidMeshTire("RL"));
            tire_RR = std::unique_ptr<ChTire>(new Generic_RigidMeshTire("RR"));
            break;
        case TireModelType::FIALA:
            tire_RL = std::unique_ptr<ChTire>(new Generic_FialaTire("RL"));
            tire_RR = std::unique_ptr<ChTire>(new Generic_FialaTire("RR"));
            break;
        default:
            std::cout << "Tire type not supported!" << std::endl;
            return 1;
    }

    tire_RL->Initialize(rear_side.GetWheelBody(FRONT_LEFT), LEFT);
    tire_RR->Initialize(rear_side.GetWheelBody(FRONT_RIGHT), RIGHT);
    tire_RL->SetVisualizationType(VisualizationType::MESH);
    tire_RR->SetVisualizationType(VisualizationType::MESH);

    // Initialize Irrlicht app
    ChWheeledVehicleIrrApp app(&front_side, &powertrain, L"Articulated Vehicle Demo");
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

    // Inter-module communication data
    TerrainForces tire_front_forces(2);
    TerrainForces tire_rear_forces(2);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();

        powertrain_torque = powertrain.GetOutputTorque();

        tire_front_forces[0] = tire_FL->GetTireForce();
        tire_front_forces[1] = tire_FR->GetTireForce();
        tire_rear_forces[0] = tire_RL->GetTireForce();
        tire_rear_forces[1] = tire_RR->GetTireForce();

        driveshaft_speed = front_side.GetDriveshaftSpeed();

        WheelState wheel_FL = front_side.GetWheelState(FRONT_LEFT);
        WheelState wheel_FR = front_side.GetWheelState(FRONT_RIGHT);

        WheelState wheel_RL = rear_side.GetWheelState(FRONT_LEFT);
        WheelState wheel_RR = rear_side.GetWheelState(FRONT_RIGHT);

        // Update modules (process inputs from other modules)
        time = front_side.GetSystem()->GetChTime();

        driver.Synchronize(time);

        terrain.Synchronize(time);

        tire_FL->Synchronize(time, wheel_FL, terrain);
        tire_FR->Synchronize(time, wheel_FR, terrain);
        tire_RL->Synchronize(time, wheel_RL, terrain);
        tire_RR->Synchronize(time, wheel_RR, terrain);

        powertrain.Synchronize(time, throttle_input, driveshaft_speed);

        front_side.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_front_forces);
        rear_side.Synchronize(time, steering_input, braking_input, tire_rear_forces);

        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);

        terrain.Advance(step_size);

        tire_FL->Advance(step_size);
        tire_FR->Advance(step_size);
        tire_RL->Advance(step_size);
        tire_RR->Advance(step_size);

        powertrain.Advance(step_size);

        front_side.Advance(step_size);

        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
