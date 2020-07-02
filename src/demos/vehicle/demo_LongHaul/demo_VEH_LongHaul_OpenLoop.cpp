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
// Authors: Rainer Gericke
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
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "subsystems/SemiTractor_tire.h"
#include "subsystems/SemiTractor_powertrain.h"
#include "subsystems/SemiTractor_vehicle.h"

#include "subsystems/SemiTrailer.h"
#include "subsystems/SemiTrailer_tire.h"

using namespace chrono;
using namespace chrono::vehicle;

// Initial vehicle position
ChVector<> initLoc(0, 0, 0.6);

// Initial vehicle orientation
double initYaw = 0;  //// CH_C_PI / 6;

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

// Tire visualisation
VisualizationType tire_vis_type = VisualizationType::MESH;

int main(int argc, char* argv[]) {
    // Create tractor and trailer
    SemiTractor_vehicle vehicle(false, ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, Q_from_AngZ(initYaw)));
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);
    auto drvLine = std::static_pointer_cast<ChShaftsDriveline4WD>(vehicle.GetDriveline());
    drvLine->LockCentralDifferential(0, false);

    SemiTrailer trailer(vehicle.GetSystem(), false);
    trailer.Initialize(vehicle.GetChassis(), ChVector<>(-4.64, 0, 0.0));
    trailer.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    trailer.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer.SetWheelVisualizationType(VisualizationType::MESH);

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
    auto powertrain = chrono_types::make_shared<SemiTractor_powertrain>("Powertrain");
    vehicle.InitializePowertrain(powertrain);

    // Create the tractor tires
    auto tire_FL = chrono_types::make_shared<SemiTractor_tire>("TractorTire_FL");
    auto tire_FR = chrono_types::make_shared<SemiTractor_tire>("TractorTire_FR");

    auto tire_RL1i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL1i");
    auto tire_RR1i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR1i");
    auto tire_RL1o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL1o");
    auto tire_RR1o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR1o");

    auto tire_RL2i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL2i");
    auto tire_RR2i = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR2i");
    auto tire_RL2o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RL2o");
    auto tire_RR2o = chrono_types::make_shared<SemiTractor_tire>("TractorTire_RR2o");

    vehicle.InitializeTire(tire_FL, vehicle.GetAxle(0)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_FR, vehicle.GetAxle(0)->m_wheels[1], tire_vis_type);

    vehicle.InitializeTire(tire_RL1i, vehicle.GetAxle(1)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_RR1i, vehicle.GetAxle(1)->m_wheels[1], tire_vis_type);
    vehicle.InitializeTire(tire_RL1o, vehicle.GetAxle(1)->m_wheels[2], tire_vis_type);
    vehicle.InitializeTire(tire_RR1o, vehicle.GetAxle(1)->m_wheels[3], tire_vis_type);

    vehicle.InitializeTire(tire_RL2i, vehicle.GetAxle(2)->m_wheels[0], tire_vis_type);
    vehicle.InitializeTire(tire_RR2i, vehicle.GetAxle(2)->m_wheels[1], tire_vis_type);
    vehicle.InitializeTire(tire_RL2o, vehicle.GetAxle(2)->m_wheels[2], tire_vis_type);
    vehicle.InitializeTire(tire_RR2o, vehicle.GetAxle(2)->m_wheels[3], tire_vis_type);

    auto tr_tire_FL = chrono_types::make_shared<SemiTrailer_tire>("FL");
    auto tr_tire_FR = chrono_types::make_shared<SemiTrailer_tire>("FR");
    auto tr_tire_ML = chrono_types::make_shared<SemiTrailer_tire>("ML");
    auto tr_tire_MR = chrono_types::make_shared<SemiTrailer_tire>("MR");
    auto tr_tire_RL = chrono_types::make_shared<SemiTrailer_tire>("RL");
    auto tr_tire_RR = chrono_types::make_shared<SemiTrailer_tire>("RR");

    // Create the tractor tires
    trailer.InitializeTire(tr_tire_FL, trailer.GetAxle(0)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_FR, trailer.GetAxle(0)->m_wheels[1], tire_vis_type);
    trailer.InitializeTire(tr_tire_ML, trailer.GetAxle(1)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_MR, trailer.GetAxle(1)->m_wheels[1], tire_vis_type);
    trailer.InitializeTire(tr_tire_RL, trailer.GetAxle(2)->m_wheels[0], tire_vis_type);
    trailer.InitializeTire(tr_tire_RR, trailer.GetAxle(2)->m_wheels[1], tire_vis_type);

    ChWheeledVehicleIrrApp app(&vehicle, L"Open Loop Road Train");

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

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int output_steps = (int)std::ceil(output_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;

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
    }

    return 0;
}
