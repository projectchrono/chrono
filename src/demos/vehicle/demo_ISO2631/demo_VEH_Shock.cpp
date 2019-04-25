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
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
// ...and specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

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

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(90, 0, 0.6);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const int heightVals[6] = {0, 50, 100, 150, 200, 250};
    int iObstacle = 1;
    double target_speed = 5.0;
    int iTire = 1;
    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidObstacle1.json");

    switch (argc) {
        default:
        case 1:
            GetLog() << "usage: demo_VEH_Shock [ObstacleNumber [Speed]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89) = " << iTire << "\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89) = " << iTire << "\n";
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89) = " << iTire << "\n";
            break;
        case 4:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
            if (atoi(argv[3]) >= 1 && atoi(argv[3]) <= 4) {
                iTire = atoi(argv[3]);
            }
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
                     << "Speed       = " << target_speed << " m/s\n"
                     << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka, 4=Pacejka89) = " << iTire << "\n";
            break;
    }
    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChMaterialSurface::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, QUNIT));
    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetStepsize(step_size);
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    GetLog() << "\nBe patient - startup may take some time... \n";

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
    powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft());

    // Create and initialize the tires
    int num_axles = vehicle.GetNumberAxles();
    int num_wheels = 2 * num_axles;

    // handling tire works, but still too high results;
    // a validated flexible tire model would be the best choice
    std::vector<std::shared_ptr<TMeasyTire> > tmeasy_tires(num_wheels);
    std::vector<std::shared_ptr<FialaTire> > fiala_tires(num_wheels);
    std::vector<std::shared_ptr<hmmwv::HMMWV_Pac02Tire> > pacejka_tires(num_wheels);
    std::vector<std::shared_ptr<hmmwv::HMMWV_Pac89Tire> > pacejka89_tires(num_wheels);
    GetLog() << "TMeasy Tire selected\n";
    for (int i = 0; i < num_wheels; i++) {
        switch (iTire) {
            default:
            case 1:
                tmeasy_tires[i] = std::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasy_tire_file));
                if (tmeasy_tires[i] == nullptr) {
                    GetLog() << "Bad TMeasy tire problem\n";
                }
                tmeasy_tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
                tmeasy_tires[i]->SetVisualizationType(VisualizationType::MESH);
                break;
            case 2:
                fiala_tires[i] = std::make_shared<FialaTire>(vehicle::GetDataFile(fiala_tire_file));
                if (fiala_tires[i] == nullptr) {
                    GetLog() << "Bad Fiala tire problem\n";
                }
                fiala_tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
                fiala_tires[i]->SetVisualizationType(VisualizationType::MESH);
            case 3:
                pacejka_tires[i] = std::make_shared<hmmwv::HMMWV_Pac02Tire>(vehicle::GetDataFile(pacejka_tire_file));
                if (pacejka_tires[i] == nullptr) {
                    GetLog() << "Bad Pacejka tire problem\n";
                }
                pacejka_tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
                pacejka_tires[i]->SetVisualizationType(VisualizationType::MESH);
                break;
            case 4:
                pacejka89_tires[i] = std::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                if (pacejka89_tires[i] == nullptr) {
                    GetLog() << "Bad Pacejka89 tire problem\n";
                }
                pacejka89_tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
                pacejka89_tires[i]->SetVisualizationType(VisualizationType::MESH);
                break;
        }
    }

    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);

#ifdef USE_IRRLICHT
    std::wstring windowTitle = L"Vehicle Shock Test Demo ";

    switch (iTire) {
        default:
        case 1:
            windowTitle.append(L"(TMeasy Tire) - " + std::to_wstring(heightVals[iObstacle]) + L" mm Obstacle Height");
            break;
        case 2:
            windowTitle.append(L"(Fiala Tire) - " + std::to_wstring(heightVals[iObstacle]) + L" mm Obstacle Height");
            break;
        case 3:
            windowTitle.append(L"(Pacejka Tire) - " + std::to_wstring(heightVals[iObstacle]) + L" mm Obstacle Height");
            break;
        case 4:
            windowTitle.append(L"(Pacejka89 Tire) - " + std::to_wstring(heightVals[iObstacle]) +
                               L" mm Obstacle Height");
            break;
    }

    ChVehicleIrrApp app(&vehicle, &powertrain, windowTitle.c_str());

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
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

    // Inter-module communication data
    TerrainForces tire_forces(num_wheels);
    WheelStates wheel_states(num_wheels);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 100.0;  // start logging when the vehicle crosses this x position
    double xend = 160.0;    // end logging here, this also the end of our world

#ifdef USE_IRRLICHT

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tire_forces[i] = tmeasy_tires[i]->GetTireForce();
                    break;
                case 2:
                    tire_forces[i] = fiala_tires[i]->GetTireForce();
                    break;
                case 3:
                    tire_forces[i] = pacejka_tires[i]->GetTireForce();
                    break;
                case 4:
                    tire_forces[i] = pacejka89_tires[i]->GetTireForce();
                    break;
            }
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tmeasy_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 2:
                    fiala_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 3:
                    pacejka_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 4:
                    pacejka89_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
            }
        }
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        powertrain.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tmeasy_tires[i]->Advance(step_size);
                    break;
                case 2:
                    fiala_tires[i]->Advance(step_size);
                    break;
                case 3:
                    pacejka_tires[i]->Advance(step_size);
                    break;
                case 4:
                    pacejka89_tires[i]->Advance(step_size);
                    break;
            }
        }
        app.Advance(step_size);

        double xpos = vehicle.GetWheelPos(0).x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            double speed = vehicle.GetVehicleSpeed();
            ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }

        app.EndScene();
    }

#else

    double v_pos;
    while ((v_pos = vehicle.GetWheelPos(0).x()) < xend) {
        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tire_forces[i] = tmeasy_tires[i]->GetTireForce();
                    break;
                case 2:
                    tire_forces[i] = fiala_tires[i]->GetTireForce();
                    break;
                case 3:
                    tire_forces[i] = pacejka_tires[i]->GetTireForce();
                    break;
                case 4:
                    tire_forces[i] = pacejka89_tires[i]->GetTireForce();
                    break;
            }
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tmeasy_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 2:
                    fiala_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 3:
                    pacejka_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
                case 4:
                    pacejka89_tires[i]->Synchronize(time, wheel_states[i], terrain, collision_type);
                    break;
            }
        }

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        powertrain.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        for (int i = 0; i < num_wheels; i++) {
            switch (iTire) {
                default:
                case 1:
                    tmeasy_tires[i]->Advance(step_size);
                    break;
                case 2:
                    fiala_tires[i]->Advance(step_size);
                    break;
                case 3:
                    pacejka_tires[i]->Advance(step_size);
                    break;
                case 4:
                    pacejka89_tires[i]->Advance(step_size);
                    break;
            }
        }

        if (v_pos >= xstart) {
            double speed = vehicle.GetVehicleSpeed();
            ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(seat_acc);
        }
    }

#endif

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
    GetLog() << "  Maximum Vertical Seat Acceleration = " << az << " g\n";
    if (az <= az_limit) {
        GetLog() << "Az <= " << az_limit << " g (ok)\n";
    } else {
        GetLog() << "Az > " << az_limit << " g - severe risk for average occupant!\n";
    }
    return 0;
}