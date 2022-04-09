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
// Main driver function for a vehicle specified through JSON files + example for
// obtaining ride quality results presented by ISO 2631-1
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
#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_PacejkaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
// specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON file for powertrain (CVT)
std::string simplepowertrain_file("generic/powertrain/SimpleCVTPowertrain.json");

// JSON files tire models
std::string tmeasy_tire_file("hmmwv/tire/HMMWV_TMeasy_converted.json");
std::string fiala_tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");
std::string pacejka_tire_file("hmmwv/tire/HMMWV_pacejka.json");

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(-100, 0, 0.6);

// Simulation step size
double step_size = 1e-3;

// Logging of seat acceleration data on flat road surface is useless and would lead to distorted results
double xstart = 0.0;  // start logging when the vehicle crosses this x position
double xend = 400.0;  // end logging here, this also the end of our world

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    int iTire = 1;

    double rmsVal = 0.0;
    int iTerrain = 1;
    double target_speed = 15.0;

    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidRandom1.json");

    switch (argc) {
        default:
        case 1:
            GetLog() << "usage: demo_VEH_Ride [TerrainNumber [Speed [TireNumberOneToFive]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iTerrain << "\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89, 5=Pacejka02, 6=Rigid) = " << iTire
                     << "\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iTerrain << "\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89, 5=Pacejka02, 6=Rigid) = " << iTire
                     << "\n";
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            target_speed = atof(argv[2]);
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iTerrain << "\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89, 5=Pacejka02, 6=Rigid) = " << iTire
                     << "\n";
            break;
        case 4:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            target_speed = atof(argv[2]);
            if (atoi(argv[3]) >= 1 && atoi(argv[3]) <= 6) {
                iTire = atoi(argv[3]);
            }
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iTerrain << "\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89, 5=Pacejka02, 6=Rigid) = " << iTire
                     << "\n";
            break;
    }

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    ChContactMethod contact_method = ChContactMethod::SMC;
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, QUNIT));
    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the ground
    RandomSurfaceTerrain terrain(vehicle.GetSystem(), xend);

    if (iTire == 6) {
        MaterialInfo minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto terrain_mat = minfo.CreateMaterial(contact_method);
        terrain.EnableCollisionMesh(terrain_mat, std::abs(initLoc.x()) + 5);
    }

    switch (iTerrain) {
        default:
        case 1:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_A_CORR, vehicle.GetWheeltrack(0));
            break;
        case 2:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_B_CORR, vehicle.GetWheeltrack(0));
            break;
        case 3:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_C_CORR, vehicle.GetWheeltrack(0));
            break;
        case 4:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_D_CORR, vehicle.GetWheeltrack(0));
            break;
        case 5:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_E_CORR, vehicle.GetWheeltrack(0));
            break;
        case 6:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_F_NOCORR, vehicle.GetWheeltrack(0));
            break;
        case 7:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_G_NOCORR, vehicle.GetWheeltrack(0));
            break;
        case 8:
            terrain.Initialize(RandomSurfaceTerrain::SurfaceType::ISO8608_H_NOCORR, vehicle.GetWheeltrack(0));
            break;
    }

    rmsVal = terrain.GetRMS() * 1000.0;  // unit [mm]

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<SimpleCVTPowertrain>(vehicle::GetDataFile(simplepowertrain_file));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        switch (iTire) {
            default:
            case 1: {
                auto tireL = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
                auto tireR = chrono_types::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 2: {
                auto tireL = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
                auto tireR = chrono_types::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 3: {
                auto tireL =
                    chrono_types::make_shared<hmmwv::HMMWV_PacejkaTire>(vehicle::GetDataFile(pacejka_tire_file));
                auto tireR =
                    chrono_types::make_shared<hmmwv::HMMWV_PacejkaTire>(vehicle::GetDataFile(pacejka_tire_file));
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 4: {
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 5: {
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_Pac02Tire>("HMMWV_Pac02_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_Pac02Tire>("HMMWV_Pac02_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 6: {
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);

                break;
            }
        }
    }

    ChISO2631_Vibration_SeatCushionLogger seat_logger(step_size);

#ifdef USE_IRRLICHT

    // Create the visualization application
    std::wstring windowTitle = L"Vehicle Ride Quality Demo ";
    switch (iTire) {
        default:
        case 1:
            windowTitle.append(L"(TMeasy Tire)");
            break;
        case 2:
            windowTitle.append(L"(Fiala Tire)");
            break;
        case 3:
            windowTitle.append(L"(Pacejka Tire)");
            break;
        case 4:
            windowTitle.append(L"(Pacejka89 Tire)");
            break;
        case 5:
            windowTitle.append(L"(Pacejka02 Tire)");
            break;
        case 6:
            windowTitle.append(L"(Rigid Tire)");
            break;
    }
    windowTitle.append(L" - " + std::to_wstring(rmsVal) + L" mm RMS");
    ChWheeledVehicleIrrApp app(&vehicle, windowTitle);

    app.GetSceneManager()->setAmbientLight(irr::video::SColorf(0.1f, 0.1f, 0.1f, 1.0f));
    app.AddLight(irr::core::vector3df(-50.f, -30.f, 40.f), 50, irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f));
    app.AddLight(irr::core::vector3df(10.f, 30.f, 40.f), 50, irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f));
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

#endif

    // Create the driver
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed, false);
    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

#ifdef USE_IRRLICHT

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        app.Advance(step_size);

        double xpos = vehicle.GetSpindlePos(0, LEFT).x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            double speed = vehicle.GetSpeed();
            ChVector<> seat_acc =
                vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed, seat_acc);
        }

        app.EndScene();
    }

#else

    double xpos;
    while ((xpos = vehicle.GetSpindlePos(0, LEFT).x()) < xend) {
        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        if (xpos >= xstart) {
            double speed = vehicle.GetSpeed();
            ChVector<> seat_acc =
                vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed, seat_acc);
        }
    }

#endif

    double ride_limit = 2.0;
    double avg_speed = seat_logger.GetAVGSpeed();
    double cf = seat_logger.GetCrestFactor();
    double awv = seat_logger.GetAW_V();
    double vdv = seat_logger.GetVDV();
    double svdv = seat_logger.GetSeverityVDV();
    double ap = seat_logger.GetAbsorbedPowerVertical();

    GetLog() << "Ride Quality Results #1 (ISO 2631-1):\n";
    GetLog() << "  Root Mean Square of the Road = " << rmsVal << " mm\n";
    GetLog() << "  Average Speed                = " << avg_speed << " m/s\n";
    GetLog() << "  Weighted Acceleration        = " << awv << " m/s^2\n";
    GetLog() << "  Vibration Dose Value         = " << vdv << " m/s^1.75\n";
    GetLog() << "  Vibration Exposure Time      = " << seat_logger.GetExposureTime() << " s\n";
    GetLog() << "  Crest Factor                 = " << cf << "\n";
    GetLog() << "  VDV based Severity Criterion = " << svdv << "\n";
    if (svdv < 1.75) {
        GetLog() << "\nVDV Severitiy < 1.75: the Weighted Acceleration AWV ist the prefered result\n";
        if (awv <= ride_limit) {
            GetLog() << "  AWV <= " << ride_limit << "m/s^2 (ok)\n";
        } else {
            GetLog() << "  AWV > " << ride_limit << "m/s^2 (above limit)\n";
        }
    } else {
        GetLog() << "\nVDV Severitiy >= 1.75: the Vibration Dose Value VDV ist the prefered result\n";
        if (vdv <= ride_limit) {
            GetLog() << "  VDV <= " << ride_limit << "m/s^1.75 (ok)\n";
        } else {
            GetLog() << "  VDV > " << ride_limit << "m/s^1.75 (above limit)\n";
        }
    }
    GetLog() << "\nRide Quality Results #2 (Absorbed Power Method):\n";
    GetLog() << "  Absorbed Power                = " << ap << " W\n";
    return 0;
}
