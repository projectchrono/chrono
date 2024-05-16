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
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RandomSurfaceTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

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

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector3d initLoc(-100, 0, 0.6);

// Simulation step size
double step_size = 1e-3;

// Logging of seat acceleration data on flat road surface is useless and would lead to distorted results
double xstart = 0.0;  // start logging when the vehicle crosses this x position
double xend = 400.0;  // end logging here, this also the end of our world

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    int iTire = 1;
    int iTerrain = 1;

    double rmsVal = 0.0;
    double target_speed = 15.0;

    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidRandom1.json");

    switch (argc) {
        default:
        case 1:
            std::cout << "usage: demo_VEH_Ride [TerrainNumber [Speed [TireNumberOneToFive]]\n\n";
            std::cout << "Using standard values for simulation:\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            target_speed = atof(argv[2]);
            break;
        case 4:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 8) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            target_speed = atof(argv[2]);
            if (atoi(argv[3]) >= 1 && atoi(argv[3]) <= 5) {
                iTire = atoi(argv[3]);
            }
            break;
    }
    std::cout << "Terrain No. = " << iTerrain << "\n"
              << "Speed       = " << target_speed << " m/s\n"
              << "Tire Code (1=TMeasy, 2=Fiala, 3=Pacejka89, 4=Pacejka02, 5=Rigid) = " << iTire << "\n";

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

    // Associate a collision system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the ground
    RandomSurfaceTerrain terrain(vehicle.GetSystem(), xend);

    if (iTire == 5) {
        ChContactMaterialData minfo;
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
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_file));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
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
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("HMMWV_Pac89_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);
                break;
            }
            case 5: {
                auto tireL = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
                auto tireR = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("HMMWV_Rigid_Tire");
                vehicle.InitializeTire(tireL, axle->m_wheels[0], VisualizationType::MESH, collision_type);
                vehicle.InitializeTire(tireR, axle->m_wheels[1], VisualizationType::MESH, collision_type);

                break;
            }
        }
    }

    ChISO2631_Vibration_SeatCushionLogger seat_logger(step_size);

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

    std::string title = "Vehicle Ride Quality Demo ";
    switch (iTire) {
        default:
        case 1:
            title.append("(TMeasy Tire)");
            break;
        case 2:
            title.append("(Fiala Tire)");
            break;
        case 3:
            title.append("(Pacejka89 Tire)");
            break;
        case 4:
            title.append("(Pacejka02 Tire)");
            break;
        case 5:
            title.append("(Rigid Tire)");
            break;
    }
    title.append(" - " + std::to_string(rmsVal) + " mm RMS");

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

    if (!vis || vis_type == ChVisualSystem::Type::NONE) {
        double xpos;
        while ((xpos = vehicle.GetSpindlePos(0, LEFT).x()) < xend) {
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

            if (xpos >= xstart) {
                double speed = vehicle.GetSpeed();
                ChVector3d seat_acc = vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
                seat_logger.AddData(speed, seat_acc);
            }
        }
    } else {
        while (vis->Run()) {
            // Render scene
            vis->BeginScene();
            vis->Render();

            // Get driver inputs
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
                double speed = vehicle.GetSpeed();
                ChVector3d seat_acc = vehicle.GetPointAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
                seat_logger.AddData(speed, seat_acc);
            }

            vis->EndScene();
        }
    }

    double ride_limit = 2.0;
    double avg_speed = seat_logger.GetAVGSpeed();
    double cf = seat_logger.GetCrestFactor();
    double awv = seat_logger.GetAW_V();
    double vdv = seat_logger.GetVDV();
    double svdv = seat_logger.GetSeverityVDV();
    double ap = seat_logger.GetAbsorbedPowerVertical();

    std::cout << "Ride Quality Results #1 (ISO 2631-1):\n";
    std::cout << "  Root Mean Square of the Road = " << rmsVal << " mm\n";
    std::cout << "  Average Speed                = " << avg_speed << " m/s\n";
    std::cout << "  Weighted Acceleration        = " << awv << " m/s^2\n";
    std::cout << "  Vibration Dose Value         = " << vdv << " m/s^1.75\n";
    std::cout << "  Vibration Exposure Time      = " << seat_logger.GetExposureTime() << " s\n";
    std::cout << "  Crest Factor                 = " << cf << "\n";
    std::cout << "  VDV based Severity Criterion = " << svdv << "\n";
    if (svdv < 1.75) {
        std::cout << "\nVDV Severitiy < 1.75: the Weighted Acceleration AWV ist the prefered result\n";
        if (awv <= ride_limit) {
            std::cout << "  AWV <= " << ride_limit << "m/s^2 (ok)\n";
        } else {
            std::cout << "  AWV > " << ride_limit << "m/s^2 (above limit)\n";
        }
    } else {
        std::cout << "\nVDV Severitiy >= 1.75: the Vibration Dose Value VDV ist the prefered result\n";
        if (vdv <= ride_limit) {
            std::cout << "  VDV <= " << ride_limit << "m/s^1.75 (ok)\n";
        } else {
            std::cout << "  VDV > " << ride_limit << "m/s^1.75 (above limit)\n";
        }
    }
    std::cout << "\nRide Quality Results #2 (Absorbed Power Method):\n";
    std::cout << "  Absorbed Power                = " << ap << " W\n";
    return 0;
}
