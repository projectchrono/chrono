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
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
using namespace chrono::postprocess;
#endif

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// =============================================================================

// Run-time visualization system (IRRLICHT, VSG, or NONE)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON files for engine and transmission
std::string engine_file("hmmwv/powertrain/HMMWV_EngineSimple.json");
std::string transmission_file("hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json");

// JSON files tire models
std::string tmeasy_tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
std::string fiala_tire_file("hmmwv/tire/HMMWV_Fiala_converted.json");

// Tire collision type
ChTire::CollisionType collision_type = ChTire::CollisionType::ENVELOPE;

// Driver input files
std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector3d initLoc(90, 0, 0.6);

// Simulation step size (should not be too high!)
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    const int heightVals[6] = {0, 50, 100, 150, 200, 250};
    int iObstacle = 1;
    double target_speed = 5.0;
    int iTire = 1;
    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidObstacle1.json");

    switch (argc) {
        default:
        case 1:
            std::cout << "usage: demo_VEH_Shock [ObstacleNumber [Speed [TireNumberOneToFive]]\n\n";
            std::cout << "Using standard values for simulation:\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 5) {
                iObstacle = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidObstacle" + std::to_string(iObstacle) + ".json";
            }
            target_speed = atof(argv[2]);
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
            break;
    }
    std::cout << "Terrain No. = " << iObstacle << " (" << heightVals[iObstacle] << " mm Obstacle Height)\n"
              << "Speed       = " << target_speed << " m/s\n"
              << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka89, 4=Pacejka89) = " << iTire << "\n";

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChContactMethod::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, QUNIT));
    ////vehicle.GetChassis()->SetFixed(true);
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    std::cout << "\nBe patient - startup may take some time... \n";

    // Associate a collision system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the ground
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    // handling tire works, but still too high results;
    // a validated flexible tire model would be the best choice
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
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
        }
    }

    ChISO2631_Shock_SeatCushionLogger seat_logger(step_size);

    // Create the driver
    auto path = ChBezierCurve::Read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);
    driver.Initialize();

    // Create the vehicle run-time visualization
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle Shock Test Demo ";

    switch (iTire) {
        default:
        case 1:
            title.append("(TMeasy Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
        case 2:
            title.append("(Fiala Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
        case 3:
            title.append("(Pacejka89 Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
        case 4:
            title.append("(Pacejka02 Tire) - " + std::to_string(heightVals[iObstacle]) + " mm Obstacle Height");
            break;
    }

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_vsg->SetWindowSize(ChVector2i(1200, 900));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
        default:
            break;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Logging of seat acceleration data on flat road surface is useless
    double xstart = 100.0;  // start logging when the vehicle crosses this x position
    double xend = 160.0;    // end logging here, this also the end of our world

    if (!vis || vis_type == ChVisualSystem::Type::NONE) {
        double v_pos;
        while ((v_pos = vehicle.GetSpindlePos(0, LEFT).x()) < xend) {
            // Driver inputs
            DriverInputs driver_inputs = driver.GetInputs();

            // Update modules (process inputs from other modules)
            double time = vehicle.GetSystem()->GetChTime();
            driver.Synchronize(time);
            vehicle.Synchronize(time, driver_inputs, terrain);
            terrain.Synchronize(time);

            // Advance simulation for one timestep for all modules
            driver.Advance(step_size);
            vehicle.Advance(step_size);
            terrain.Advance(step_size);

            if (v_pos >= xstart) {
                ChVector3d seat_acc = vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
                seat_logger.AddData(seat_acc);
            }
        }
    } else {
        while (vis->Run()) {
            // Render scene
            vis->BeginScene();
            vis->Render();

            // Driver inputs
            DriverInputs driver_inputs = driver.GetInputs();

            // Update modules (process inputs from other modules)
            double time = vehicle.GetSystem()->GetChTime();
            driver.Synchronize(time);
            vehicle.Synchronize(time, driver_inputs, terrain);
            terrain.Synchronize(time);
            vis->Synchronize(time, driver_inputs);

            // Advance simulation for one timestep for all modules
            driver.Advance(step_size);
            vehicle.Advance(step_size);
            terrain.Advance(step_size);
            vis->Advance(step_size);

            double xpos = vehicle.GetSpindlePos(0, LEFT).x();
            if (xpos >= xend) {
                break;
            }
            if (xpos >= xstart) {
                ChVector3d seat_acc = vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
                seat_logger.AddData(seat_acc);
            }

            vis->EndScene();
        }
    }

    double se_low = 0.5;
    double se_high = 0.8;
    double se = seat_logger.GetSe();

    double az_limit = 2.5;
    double az = seat_logger.GetLegacyAz();

    std::cout << "Shock Simulation Results #1 (ISO 2631-5 Method):\n";
    std::cout << "  Significant Speed                        Vsig = " << target_speed << " m/s\n";
    std::cout << "  Equivalent Static Spine Compressive Stress Se = " << se << " MPa\n";
    if (se <= se_low) {
        std::cout << "Se <= " << se_low << " MPa (ok) - low risc of health effect, below limit for average occupants\n";
    } else if (se >= se_high) {
        std::cout << "Se >= " << se_high << " MPa - severe risc of health effect!\n";
    } else {
        std::cout << "Se is between [" << se_low << ";" << se_high << "] - risc of health effects, above limit!\n";
    }
    std::cout << "\nShock Simulation Results #2 (Traditional NRMM Method):\n";
    std::cout << "  Maximum Vertical Seat Acceleration = " << az << " g\n";
    if (az <= az_limit) {
        std::cout << "Az <= " << az_limit << " g (ok)\n";
    } else {
        std::cout << "Az > " << az_limit << " g - severe risk for average occupant!\n";
    }
    return 0;
}
