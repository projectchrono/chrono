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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Main driver function for an semitractor/semitrailer road train.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/kraz/Kraz.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::kraz;

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.6);

// Initial vehicle orientation
double initYaw = 0;  //// CH_C_PI / 6;

// Rigid terrain dimensions
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

int main(int argc, char* argv[]) {
    // Create the semi-trailer truck
    Kraz truck;
    truck.SetContactMethod(ChContactMethod::NSC);
    truck.SetChassisFixed(false);
    truck.SetInitPosition(ChCoordsys<>(initLoc, Q_from_AngZ(initYaw)));
    truck.SetTireStepSize(tire_step_size);
    truck.SetInitFwdVel(0.0);

    truck.Initialize();

    truck.SetChassisVisualizationType(VisualizationType::MESH, VisualizationType::PRIMITIVES);
    truck.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    truck.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
    truck.SetWheelVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
    truck.SetTireVisualizationType(VisualizationType::MESH, VisualizationType::MESH);

    // Create the terrain
    RigidTerrain terrain(truck.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.5f, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Semi-trailer truck :: Open Loop");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    truck.GetTractor().SetVisualSystem(vis);

    ChIrrGuiDriver driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;

    while (vis->Run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->DrawAll();
            vis->EndScene();
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = truck.GetSystem()->GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        truck.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        truck.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
