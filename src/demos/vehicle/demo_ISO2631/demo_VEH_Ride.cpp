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

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_vehicle/ChConfigVehicle.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

// Specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT

// =============================================================================

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::vehicle;

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");

// JSON file for powertrain (simple)
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// JSON files tire models
std::string tmeasytire_file("hmmwv/tire/HMMWV_TMeasyTire.json");

std::string path_file("paths/straightOrigin.txt");
std::string steering_controller_file("hmmwv/SteeringController.json");
std::string speed_controller_file("hmmwv/SpeedController.json");

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.6);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size
double step_size = 2e-3;

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2018 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const int rmsVals[5] = {0, 10, 20, 30, 40};
    int iTerrain = 1;
    double target_speed = 15.0;
    // JSON files for terrain
    std::string rigidterrain_file("terrain/RigidRandom1.json");

    switch (argc) {
        default:
        case 1:
            GetLog() << "usage: demo_VEH_Ride [TerrainNumber [Speed]]\n\n";
            GetLog() << "Using standard values for simulation:\n"
                     << "Terrain No. = " << iTerrain << " (" << rmsVals[iTerrain] << " mm RMS)\n"
                     << "Speed       = " << target_speed << " m/s\n";
            break;
        case 2:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 4) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iTerrain << " (" << rmsVals[iTerrain] << " mm RMS)\n"
                     << "Speed       = " << target_speed << " m/s\n";
            break;
        case 3:
            if (atoi(argv[1]) >= 1 && atoi(argv[1]) <= 4) {
                iTerrain = atoi(argv[1]);
                rigidterrain_file = "terrain/RigidRandom" + std::to_string(iTerrain) + ".json";
            }
            target_speed = atof(argv[2]);
            GetLog() << "Using values for simulation:\n"
                     << "Terrain No. = " << iTerrain << " (" << rmsVals[iTerrain] << " mm RMS)\n"
                     << "Speed       = " << target_speed << " m/s\n";
            break;
    }

    // --------------------------
    // Create the various modules
    // --------------------------

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChMaterialSurface::NSC);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
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
    std::vector<std::shared_ptr<TMeasyTire> > tires(num_wheels);
    for (int i = 0; i < num_wheels; i++) {
        tires[i] = std::make_shared<TMeasyTire>(vehicle::GetDataFile(tmeasytire_file));
        tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
        tires[i]->SetVisualizationType(VisualizationType::MESH);
    }

    ChISO2631_Vibration_SeatCushionLogger seat_logger(step_size);

#ifdef USE_IRRLICHT

    // Create the visualization application
    std::wstring windowTitle = L"Vehicle Ride Quality Demo - " + std::to_wstring(rmsVals[iTerrain]) + L" mm RMS";
    ChVehicleIrrApp app(&vehicle, &powertrain, windowTitle.c_str());

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

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

    // Logging of seat acceleration data on flat road surface is useless and would lead to distorted results
    double xstart = 100.0;  // start logging when the vehicle crosses this x position
    double xend = 400.0;    // end logging here, this also the end of our world

#ifdef USE_IRRLICHT

    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();
        double powertrain_torque = powertrain.GetOutputTorque();
        double driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Synchronize(time, wheel_states[i], terrain);
        // app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        terrain.Advance(step);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step);
        app.Advance(step);

        double xpos = vehicle.GetWheelPos(0).x();
        if (xpos >= xend) {
            break;
        }
        if (xpos >= xstart) {
            double speed = vehicle.GetVehicleSpeed();
            ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
            seat_logger.AddData(speed, seat_acc);
        }

        app.EndScene();
    }

#else

    double xpos;
    while ((xpos = vehicle.GetWheelPos(0).x()) < xend) {
        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();
        double powertrain_torque = powertrain.GetOutputTorque();
        double driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Synchronize(time, wheel_states[i], terrain);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        powertrain.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step_size);

        if (xpos >= xstart) {
            double speed = vehicle.GetVehicleSpeed();
            ChVector<> seat_acc = vehicle.GetVehicleAcceleration(vehicle.GetChassis()->GetLocalDriverCoordsys().pos);
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

    GetLog() << "Ride Quality Results:\n";
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
    return 0;
}