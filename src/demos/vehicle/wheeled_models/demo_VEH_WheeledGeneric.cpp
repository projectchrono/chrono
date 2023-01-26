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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Main driver function for a generic vehicle, using rigid tire-terrain contact.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/generic/Generic_Vehicle.h"
#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_FuncDriver.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
// specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

// DEBUGGING:  Uncomment the following line to print shock data
//#define DEBUG_LOG

using namespace chrono;
#ifdef USE_IRRLICHT
using namespace chrono::irrlicht;
#endif
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
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 1;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 20.0;

// Output directories (Povray only)
const std::string out_dir = GetChronoOutputPath() + "GENERIC_VEHICLE";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle: specify if chassis is fixed, the suspension type
    // and visualization mode for the various vehicle components.
    Generic_Vehicle vehicle(false, SuspensionTypeWV::MACPHERSON_STRUT, ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<Generic_SimplePowertrain>("powertrain");
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

#ifdef USE_IRRLICHT

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Generic Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);

    ChInteractiveDriverIRR driver(*vis);

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

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;

#ifdef USE_IRRLICHT

    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        // Update the position of the shadow mapping so that it follows the car
        ////if (do_shadows) {
        ////  ChVector<> lightaim = vehicle.GetChassisPos();
        ////  ChVector<> lightpos = vehicle.GetChassisPos() + ChVector<>(10, 30, 60);
        ////  irr::core::vector3df mlightpos((irr::f32)lightpos.x, (irr::f32)lightpos.y, (irr::f32)lightpos.z);
        ////  irr::core::vector3df mlightaim((irr::f32)lightaim.x, (irr::f32)lightaim.y, (irr::f32)lightaim.z);
        ////  application.GetEffects()->getShadowLight(0).setPosition(mlightpos);
        ////  application.GetEffects()->getShadowLight(0).setTarget(mlightaim);
        ////  mlight->setPosition(mlightpos);
        ////}

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

#ifdef DEBUG_LOG
        // Number of simulation steps between two output frames
        int output_steps = (int)std::ceil(output_step_size / step_size);

        if (step_number % output_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            vehicle.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
        }
#endif

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

#else

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    int render_frame = 0;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

    char filename[100];

    while (time < tend) {
        if (step_number % render_steps == 0) {
            // Output render data
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteVisualizationAssets(vehicle.GetSystem(), filename);
            std::cout << "Output frame:   " << render_frame << std::endl;
            std::cout << "Sim frame:      " << step_number << std::endl;
            std::cout << "Time:           " << time << std::endl;
            std::cout << "             throttle: " << driver.GetThrottle() << " steering: " << driver.GetSteering()
                      << std::endl;
            std::cout << std::endl;
            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);

        // Increment frame number
        step_number++;
    }

#endif

    return 0;
}
