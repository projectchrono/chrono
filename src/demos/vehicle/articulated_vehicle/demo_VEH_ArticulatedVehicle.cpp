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
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "subsystems/ACV_Vehicle.h"
#include "subsystems/ACV_EngineSimple.h"
#include "subsystems/ACV_AutomaticTransmissionSimple.h"
#include "subsystems/ACV_RigidTire.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Initial front_side position
ChVector<> initLoc(10, 0, 0.5);

// Initial front_side orientation
ChQuaternion<> initRot = Q_from_AngZ(CH_C_PI / 3);

// Type of tire model (RIGID, RIGID_MESH, or FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle
    ACV_Vehicle vehicle(false, ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create a collsion system and associate it with the underlying Chrono system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    //vehicle.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch =
        terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(0, 0, terrainHeight), QUNIT), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.5f, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the powertrain subsystems
    auto engine = chrono_types::make_shared<ACV_EngineSimple>("Engine");
    auto transmission = chrono_types::make_shared<ACV_AutomaticTransmissionSimple>("Transmission");
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the front and rear tires
    auto tire_FL = chrono_types::make_shared<ACV_RigidTire>("FL");
    auto tire_FR = chrono_types::make_shared<ACV_RigidTire>("FR");
    auto tire_RL = chrono_types::make_shared<ACV_RigidTire>("RL");
    auto tire_RR = chrono_types::make_shared<ACV_RigidTire>("RR");

    vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[0], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[1], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_RL, vehicle.GetAxle(1)->m_wheels[0], VisualizationType::PRIMITIVES);
    vehicle.InitializeTire(tire_RR, vehicle.GetAxle(1)->m_wheels[1], VisualizationType::PRIMITIVES);

    // Create the Irrlicht visualization
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Articulated Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    // Initialize interactive driver
    ChInteractiveDriverIRR driver(*vis);
    driver.SetSteeringDelta(0.04);
    driver.SetThrottleDelta(0.2);
    driver.SetBrakingDelta(0.5);

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        double time = vehicle.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
