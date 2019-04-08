// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// HMMWV acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE, SIMPLE_CVT)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::AWD;

// Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89, TMEASY)
TireModelType tire_model = TireModelType::FIALA;

// Terrain length (X direction)
double terrainLength = 300.0;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    my_hmmwv.Initialize();

    // Set subsystem visualization mode
    VisualizationType tire_vis_type =
        (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(terrainLength, 5, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 5);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"HMMWV acceleration test");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);

    // ----------------------------------------------
    // Create the straight path and the driver system
    // ----------------------------------------------

    auto path = StraightLinePath(ChVector<>(-terrainLength / 2, 0, 0.5), ChVector<>(terrainLength / 2, 0, 0.5), 1);
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ---------------------------------------------
    // Finalize construction of visualization assets
    // ---------------------------------------------

    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunction_Recorder speed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    bool done = false;

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        double speed = speed_filter.Add(my_hmmwv.GetVehicle().GetVehicleSpeed());
        if (!done) {
            speed_recorder.AddPoint(time, speed);
            if (time > 1 && std::abs((speed - last_speed) / step_size) < 0.0005) {
                done = true;
                std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                postprocess::ChGnuPlot gplot;
                gplot.SetGrid();
                gplot.SetLabelX("time (s)");
                gplot.SetLabelY("speed (m/s)");
                gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
            }
        }
        last_speed = speed;

        // End simulation
        if (time >= 100)
            break;

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        if (done) {
            throttle_input = 0.1;
            braking_input = 0.8;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("Acceleration test", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    return 0;
}
