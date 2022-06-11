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

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/AIDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

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
    Sedan my_sedan;
    my_sedan.SetContactMethod(ChContactMethod::SMC);
    my_sedan.SetChassisCollisionType(CollisionType::NONE);
    my_sedan.SetChassisFixed(false);
    my_sedan.SetInitPosition(ChCoordsys<>(ChVector<>(-30, -30, 1.0), Q_from_AngZ(CH_C_PI / 4)));
    my_sedan.SetTireType(TireModelType::TMEASY);
    my_sedan.Initialize();

    my_sedan.SetChassisVisualizationType(VisualizationType::MESH);
    my_sedan.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetWheelVisualizationType(VisualizationType::MESH);
    my_sedan.SetTireVisualizationType(VisualizationType::MESH);

    // Create the terrain
    RigidTerrain terrain(my_sedan.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);

    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100.0, 100.0);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Sedan AI Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    my_sedan.GetVehicle().SetVisualSystem(vis);

    // Create the driver system
    // Option 1: use concrete driver class
    Sedan_AIDriver driver(my_sedan.GetVehicle());
    // Option 2: use driuver specified through JSON file
    ////AIDriver driver(my_sedan.GetVehicle(), vehicle::GetDataFile("sedan/driver/Sedan_AIDriver.json"));

    driver.Initialize();

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 2e-3;

    while (vis->Run()) {
        double time = my_sedan.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->DrawAll();
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
        my_sedan.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_sedan.Advance(step_size);
        vis->Advance(step_size);

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
