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
// Demonstration of an OpenCRG terrain and the ChHumanDirver
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChHumanDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

#define READ_JSON_FILE

// =============================================================================
// Problem parameters

// Type of tire model (LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// OpenCRG input file
std::string crg_road_file = "terrain/crg_roads/Barber.crg";

// Road visualization (mesh or boundary lines)
bool useMesh = false;

// Desired minimal vehicle speed (m/s)
double minimum_speed = 12;

// Desired minimal vehicle speed (m/s)
double maximum_speed = 30;

// Simulation step size
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Output frame images
bool output_images = false;
double fps = 60;
const std::string out_dir = GetChronoOutputPath() + "OPENCRG_DEMO";
const std::string json_file = "hmmwv/driver/HumanController.json";

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
    double road_width = terrain.GetWidth();

#ifdef READ_JSON_FILE
    ChHumanDriver driver(vehicle::GetDataFile(json_file), my_hmmwv.GetVehicle(), path, "my_path", path_is_closed,
                         road_width, my_hmmwv.GetVehicle().GetMaxSteeringAngle(), 3.2);
#else
    ChHumanDriver driver(my_hmmwv.GetVehicle(), path, "my_path", path_is_closed, road_width,
                         my_hmmwv.GetVehicle().GetMaxSteeringAngle(), 3.2);
    driver.SetPreviewTime(0.5);
    driver.SetLateralGains(0.1, 2);
    driver.SetLongitudinalGains(0.1, 0.1, 0.2);
    driver.SetSpeedRange(minimum_speed, maximum_speed);
#endif
    // driver.GetSteeringController().SetGains(0.1, 5);
    // driver.GetSteeringController().SetPreviewTime(0.5);
    // driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"OpenCRG Demo Simple Realistic Human Driver",
                               irr::core::dimension2d<irr::u32>(800, 640));
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
    double t_end = 300.0;

    std::cout << "Road length:     " << road_length << std::endl;
    std::cout << "Road width:      " << road_width << std::endl;
    std::cout << "Closed loop?     " << path_is_closed << std::endl;
    std::cout << "Set end time to: " << t_end << std::endl;

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    int sim_frame = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {
        double time = my_hmmwv.GetSystem()->GetChTime();
        if (time >= t_end)
            break;

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver.GetSentinelLocation();
        const ChVector<>& pT = driver.GetTargetLocation();
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
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment simulation frame number
        sim_frame++;

        app.EndScene();
    }

    GetLog() << "Traveled Distance = " << driver.GetTraveledDistance() << " m\n";
    GetLog() << "Average Speed = " << driver.GetAverageSpeed() << " m/s\n";
    GetLog() << "Max. Speed = " << driver.GetMaxSpeed() << " m/s\n";
    GetLog() << "Min. Speed = " << driver.GetMinSpeed() << " m/s\n";
    GetLog() << "Max. Lateral Acc. = " << driver.GetMaxLatAcc() << " m^2/s\n";
    GetLog() << "Min. Lateral Acc. = " << driver.GetMinLatAcc() << " m^2/s\n";
    return 0;
}
