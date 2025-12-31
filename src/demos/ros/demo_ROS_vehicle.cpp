// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Demo to show the use of Chrono::Vehicle with ROS
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
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"

#include <chrono>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;

// =============================================================================

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;
    virtual ChContactMethod ContactMethod() const = 0;
};

class Audi_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Audi"; }
    virtual std::string VehicleJSON() const override { return "audi/json/audi_Vehicle.json"; }
    virtual std::string TireJSON() const override {
        ////return "audi/json/audi_TMeasyTire.json";
        return "audi/json/audi_Pac02Tire.json";
        ////return "audi/json/audi_RigidTire.json.json";
    }
    virtual std::string EngineJSON() const override { return "audi/json/audi_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "audi/json/audi_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
    virtual ChContactMethod ContactMethod() const override { return ChContactMethod::SMC; }
};

// Current vehicle model selection
auto vehicle_model = Audi_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation
ChVector3d initLoc(0, 0, 0.5);
double initYaw = 20 * CH_DEG_TO_RAD;

// Simulation step size
double step_size = 2e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the vehicle system
    WheeledVehicle vehicle(GetVehicleDataFile(vehicle_model.VehicleJSON()), vehicle_model.ContactMethod());
    vehicle.Initialize(ChCoordsys<>(initLoc, QuatFromAngleZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(GetVehicleDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(vehicle_model.TireJSON()));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto system = vehicle.GetSystem();

    // Create the terrain
    RigidTerrain terrain(system, GetVehicleDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle);

    // ------------

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 25;
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);

    // Create a publisher for the vehicle state
    auto vehicle_state_rate = 25;
    auto vehicle_state_topic_name = "~/output/vehicle/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
    ros_manager->RegisterHandler(vehicle_state_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    double t_end = 30;
    double time = 0;

    // Simulation loop
    vehicle.EnableRealtime(true);
    while (time < t_end) {
        // Get driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        if (!ros_manager->Update(time, step_size))
            break;
    }

    return 0;
}
