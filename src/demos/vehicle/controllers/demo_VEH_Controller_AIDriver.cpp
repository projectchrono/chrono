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
// Demo for ChAIDriver using the Sedan vehicle model
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/AIDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/sedan/Sedan_AIDriver.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize
    Sedan sedan;
    sedan.SetContactMethod(ChContactMethod::SMC);
    sedan.SetChassisCollisionType(CollisionType::NONE);
    sedan.SetChassisFixed(false);
    sedan.SetInitPosition(ChCoordsys<>(ChVector<>(-30, -30, 1.0), Q_from_AngZ(CH_C_PI / 4)));
    sedan.SetTireType(TireModelType::TMEASY);
    sedan.Initialize();

    sedan.SetChassisVisualizationType(VisualizationType::MESH);
    sedan.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    sedan.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    sedan.SetWheelVisualizationType(VisualizationType::MESH);
    sedan.SetTireVisualizationType(VisualizationType::MESH);

    // Associate a collision system
    sedan.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
    RigidTerrain terrain(sedan.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Sedan AI Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&sedan.GetVehicle());

    // Create the driver system
    // Option 1: use concrete driver class
    Sedan_AIDriver driver(sedan.GetVehicle());
    // Option 2: use driuver specified through JSON file
    ////AIDriver driver(sedan.GetVehicle(), vehicle::GetDataFile("sedan/driver/Sedan_AIDriver.json"));

    driver.Initialize();

    // Simulation loop
    sedan.GetVehicle().EnableRealtime(true);
    double step_size = 2e-3;

    while (vis->Run()) {
        double time = sedan.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Mimic controls from an Autonomy Stack
        double long_acc;     // longitudinal acceleration [m/s2]
        double wheel_angle;  // front wheel angle [rad]
        if (time < 2) {
            long_acc = 0;
            wheel_angle = 0;
        } else if (time < 4) {
            long_acc = 5;
            wheel_angle = 0;
        } else {
            long_acc = 0;
            wheel_angle = 0.4;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time, long_acc, wheel_angle, 0.0);
        DriverInputs driver_inputs = driver.GetInputs();
        terrain.Synchronize(time);
        sedan.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        sedan.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
