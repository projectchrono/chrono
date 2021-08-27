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
// Main driver function for a mrole specified through JSON files + example for
// obtaining shock effect results presented by ISO 2631-5
//
// Halfround shaped obstacles
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/mrole/mrole.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
    // specify whether the demo should actually use Irrlicht
    #define USE_IRRLICHT
#endif

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;
using namespace chrono::vehicle::mrole;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON file for powertrain (simple)
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// JSON files tire models
std::string tmeasy_tire_file("hmmwv/tire/HMMWV_TMeasy_converted.json");
std::string fiala_tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");
std::string pacejka_tire_file("hmmwv/tire/HMMWV_pacejka.json");

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(-40, 0, 0.7);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

const double mph_to_ms = 0.44704;

// Road visualization (mesh or boundary lines)
bool useMesh = true;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const int heightVals[16] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30};
    int iObstacle = 1;
    double target_speed = 5.0;
    // CRG files for terrain
    std::string crg_terrain_file("terrain/crg_roads/halfround_2in.crg");

    switch (argc) {
        default:
        case 1:
            GetLog() << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
        case 2:
            iObstacle = ChClamp(atoi(argv[1]), 1, 15);
            GetLog() << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
        case 3:
            iObstacle = ChClamp(atoi(argv[1]), 1, 15);
            target_speed = ChClamp<double>(atof(argv[2]), 1, 100);
            GetLog() << "usage: demo_VEH_Shock [ObstacleNumber [Speed_mph]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " in Obstacle Height)\n"
                     << "Speed       = " << target_speed << " mph\n";
            break;
    }
    switch (iObstacle) {
        case 2:
            crg_terrain_file = "terrain/crg_roads/halfround_4in.crg";
            break;
        case 3:
            crg_terrain_file = "terrain/crg_roads/halfround_6in.crg";
            break;
        case 4:
            crg_terrain_file = "terrain/crg_roads/halfround_8in.crg";
            break;
        case 5:
            crg_terrain_file = "terrain/crg_roads/halfround_10in.crg";
            break;
        case 6:
            crg_terrain_file = "terrain/crg_roads/halfround_12in.crg";
            break;
        case 7:
            crg_terrain_file = "terrain/crg_roads/halfround_14in.crg";
            break;
        case 8:
            crg_terrain_file = "terrain/crg_roads/halfround_16in.crg";
            break;
        case 9:
            crg_terrain_file = "terrain/crg_roads/halfround_18in.crg";
            break;
        case 10:
            crg_terrain_file = "terrain/crg_roads/halfround_20in.crg";
            break;
        case 11:
            crg_terrain_file = "terrain/crg_roads/halfround_22in.crg";
            break;
        case 12:
            crg_terrain_file = "terrain/crg_roads/halfround_24in.crg";
            break;
        case 13:
            crg_terrain_file = "terrain/crg_roads/halfround_26in.crg";
            break;
        case 14:
            crg_terrain_file = "terrain/crg_roads/halfround_28in.crg";
            break;
        case 15:
            crg_terrain_file = "terrain/crg_roads/halfround_30in.crg";
            break;
    }
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    mrole_Full mrole;
    mrole.SetContactMethod(ChContactMethod::NSC);
    mrole.SetChassisFixed(false);
    mrole.SetInitPosition(ChCoordsys<>(initLoc, QUNIT));
    mrole.SetTireType(tire_model);
    mrole.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    mrole.SetInitFwdVel(target_speed * mph_to_ms);
    mrole.Initialize();

    mrole.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mrole.SetWheelVisualizationType(VisualizationType::NONE);
    mrole.SetTireVisualizationType(VisualizationType::MESH);
    ////mrole.GetChassis()->SetFixed(true);

    GetLog() << "\nBe patient - startup may take some time... \n";

    // Create the ground
    // RigidTerrain terrain(mrole.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    CRGTerrain terrain(mrole.GetSystem());
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.Initialize(vehicle::GetDataFile(crg_terrain_file));

    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);

#ifdef USE_IRRLICHT
    std::wstring windowTitle = L"Multi Role Vehicle Shock Test";

    ChWheeledVehicleIrrApp app(&mrole.GetVehicle(), windowTitle);

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();
#endif

    // Create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(mrole.GetVehicle(), vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 40.0;  // start logging when the mrole crosses this x position
    double xend = 100.0;   // end logging here, this also the end of our world

#ifdef USE_IRRLICHT

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = mrole.GetSystem()->GetChTime();
        driver.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        mrole.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        double xpos = mrole.GetVehicle().GetVehiclePos().x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            ChVector<> seat_acc = mrole.GetVehicle().GetVehiclePointAcceleration(
                mrole.GetVehicle().GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }

        app.EndScene();
    }

#else
    const double ms_to_mph = 2.2369362921;

    double v_pos;
    while ((v_pos = mrole.GetVehicle().GetVehiclePos().x()) < xend) {
        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = mrole.GetSystem()->GetChTime();
        driver.Synchronize(time);
        mrole.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        mrole.Advance(step_size);
        terrain.Advance(step_size);

        if (v_pos >= xstart) {
            double speed = mrole.GetVehicleSpeed();
            ChVector<> seat_acc = mrole.GetVehiclePointAcceleration(mrole.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }
    }

#endif

    const double ms_to_mph = 2.2369362921;
    double se_low = 0.5;
    double se_high = 0.8;
    double se = seat_logger.GetSe();

    double az_limit = 2.5;
    double az = seat_logger.GetLegacyAz();

    GetLog() << "Shock Simulation Results #1 (ISO 2631-5 Method):\n";
    GetLog() << "  Significant Speed                        Vsig = " << target_speed << " m/s\n";
    GetLog() << "  Equivalent Static Spine Compressive Stress Se = " << se << " MPa\n";
    if (se <= se_low) {
        GetLog() << "Se <= " << se_low << " MPa (ok) - low risc of health effect, below limit for average occupants\n";
    } else if (se >= se_high) {
        GetLog() << "Se >= " << se_high << " MPa - severe risc of health effect!\n";
    } else {
        GetLog() << "Se is between [" << se_low << ";" << se_high << "] - risc of health effects, above limit!\n";
    }
    GetLog() << "\nShock Simulation Results #2 (Traditional NRMM Method):\n";
    GetLog() << "Significant Speed = " << target_speed << " mph\n";
    GetLog() << "Obstacle Height   = " << heightVals[iObstacle] << " in\n";
    GetLog() << "  Maximum Vertical Seat Acceleration = " << az << " g\n";
    if (az <= az_limit) {
        GetLog() << "Az <= " << az_limit << " g (ok)\n";
    } else {
        GetLog() << "Az > " << az_limit << " g - severe risk for average occupant!\n";
    }
    return 0;
}