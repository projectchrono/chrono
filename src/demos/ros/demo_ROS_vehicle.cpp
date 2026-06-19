// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: integrating ROS with Chrono::Vehicle. A JSON-specified wheeled vehicle
// (Audi) is driven from ROS (chrono_ros_interfaces/msg/DriverInputs) and its
// chassis state is published. Runs headless (no run-time window).
//
//   ros2 topic echo /chrono_ros_node/output/vehicle/state/pose
//   ros2 topic pub  /chrono_ros_node/input/driver_inputs ...
//
// =============================================================================

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;

// JSON specification for an Audi wheeled vehicle.
static const std::string vehicle_json = "audi/json/audi_Vehicle.json";
static const std::string tire_json = "audi/json/audi_Pac02Tire.json";
static const std::string engine_json = "audi/json/audi_EngineSimpleMap.json";
static const std::string transmission_json = "audi/json/audi_AutomaticTransmissionSimpleMap.json";
static const std::string rigidterrain_json = "terrain/RigidPlane.json";

ChVector3d initLoc(0, 0, 0.5);
double initYaw = 20 * CH_DEG_TO_RAD;
double step_size = 2e-3;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Wheeled vehicle from JSON.
    WheeledVehicle vehicle(GetVehicleDataFile(vehicle_json), ChContactMethod::SMC);
    vehicle.Initialize(ChCoordsys<>(initLoc, QuatFromAngleZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    auto engine = ReadEngineJSON(GetVehicleDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    RigidTerrain terrain(vehicle.GetSystem(), GetVehicleDataFile(rigidterrain_json));
    terrain.Initialize();

    auto driver = std::make_shared<ChDriver>(vehicle);

    // ------------ ROS: clock + DriverInputs subscriber + chassis state.
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    ros_manager->RegisterHandler(
        chrono_types::make_shared<ChROSDriverInputsHandler>(25, driver, "~/input/driver_inputs"));
    ros_manager->RegisterHandler(
        chrono_types::make_shared<ChROSBodyHandler>(25, vehicle.GetChassisBody(), "~/output/vehicle/state"));
    ros_manager->Initialize();

    // ------------ Simulation loop (real-time paced by the vehicle).
    double t_end = 30;
    double time = 0;

    vehicle.EnableRealtime(true);
    while (time < t_end) {
        DriverInputs driver_inputs = driver->GetInputs();

        time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        if (!ros_manager->Update(time, step_size))
            break;
    }

    return 0;
}
