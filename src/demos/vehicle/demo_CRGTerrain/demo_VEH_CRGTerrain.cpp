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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Demonstration of an OpenCRG terrain.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Select Path Follower, uncomment the next line to get the pure PID driver
//#define USE_PID 1
// to use the XT controller uncomment the next line
//#define USE_XT
// to use the SR controller uncomment the next line
#define USE_SR
// =============================================================================
// Problem parameters

// Type of tire model (LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// OpenCRG input file
std::string crg_road_file = "terrain/crg_roads/Barber.crg";
////std::string crg_road_file = "terrain/crg_roads/Horstwalde.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_arc.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_banked.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_circle.crg";
////std::string crg_road_file = "terrain/crg_roads/handmade_sloped.crg";

// Road visualization (mesh or boundary lines)
bool useMesh = false;

// Desired vehicle speed (m/s)
double target_speed = 12;

// Simulation step size
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Output frame images
bool output_images = false;
double fps = 60;
const std::string out_dir = GetChronoOutputPath() + "OPENCRG_DEMO";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ---------------------------------------
    // Create the vehicle, terrain, and driver
    // ---------------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(2, 0, 0.5), QUNIT));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain
    CRGTerrain terrain(my_hmmwv.GetSystem());
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_road_file));

    // Get the vehicle path (middle of the road)
    auto path = terrain.GetPath();
    bool path_is_closed = terrain.IsPathClosed();
    double road_length = terrain.GetLength();

#ifdef USE_PID
    // Create the driver system based on PID steering controller
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed, path_is_closed);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"OpenCRG Demo PID Steering",
                        irr::core::dimension2d<irr::u32>(800, 640));
#endif
#ifdef USE_XT
    // Create the driver system based on XT steering controller
    ChPathFollowerDriverXT driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed, path_is_closed,
                                  my_hmmwv.GetVehicle().GetMaxSteeringAngle());
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.4, 1, 1, 1);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"OpenCRG Demo XT Steering",
                        irr::core::dimension2d<irr::u32>(800, 640));
#endif
#ifdef USE_SR
    ChPathFollowerDriverSR driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed, path_is_closed,
                                  my_hmmwv.GetVehicle().GetMaxSteeringAngle(), 3.2);
    driver.GetSteeringController().SetGains(0.1, 5);
    driver.GetSteeringController().SetPreviewTime(0.5);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"OpenCRG Demo SR Steering",
                        irr::core::dimension2d<irr::u32>(800, 640));
#endif
    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ----------------
    // Output directory
    // ----------------

    if (output_images) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Final time
    double t_end = 2 + road_length / target_speed;
    if (path_is_closed) {
        t_end += 30.0;
    }
    std::cout << "Road length:     " << road_length << std::endl;
    std::cout << "Closed loop?     " << path_is_closed << std::endl;
    std::cout << "Set end time to: " << t_end << std::endl;

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        // Render scene and output images
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        if (output_images && sim_frame % render_steps == 0) {
            char filename[200];
            sprintf(filename, "%s/image_%05d.bmp", out_dir.c_str(), render_frame++);
            app.WriteImageToFile(filename);
            render_frame++;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        // Increment simulation frame number
        sim_frame++;

        app.EndScene();
    }

    return 0;
}
