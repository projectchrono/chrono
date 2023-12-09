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
// Authors: Aaron Young
// =============================================================================
//
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include <chrono>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::synchrono;
using namespace chrono::vehicle;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of tire model
TireModelType tire_model = TireModelType::TMEASY;

// Type of vehicle
enum VehicleType { SEDAN, HMMWV, UAZ, CITYBUS, MAN };

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step sizes
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);
void GetVehicleModelFiles(VehicleType type,
                          std::string& vehicle,
                          std::string& engine,
                          std::string& transmission,
                          std::string& tire,
                          std::string& zombie,
                          double& cam_distance);

class IrrAppWrapper {
  public:
    IrrAppWrapper(std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> app = nullptr) : m_app(app) {}

    void Synchronize(double time, const DriverInputs& driver_inputs) {
        if (m_app)
            m_app->Synchronize(time, driver_inputs);
    }

    void Advance(double step) {
        if (m_app)
            m_app->Advance(step);
    }

    void Render() {
        if (m_app) {
            m_app->BeginScene();
            m_app->Render();
            m_app->EndScene();
        }
    }

    void Set(std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> app) { m_app = app; }
    bool IsOk() { return m_app ? m_app->GetDevice()->run() : true; }

    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> m_app;
};

class DriverWrapper : public ChDriver {
  public:
    DriverWrapper(ChVehicle& vehicle) : ChDriver(vehicle) {}

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override {
        if (irr_driver) {
            irr_driver->Synchronize(time);
            m_throttle = irr_driver->GetThrottle();
            m_steering = irr_driver->GetSteering();
            m_braking = irr_driver->GetBraking();
        }
    }

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) override {
        if (irr_driver)
            irr_driver->Advance(step);
    }

    void Set(std::shared_ptr<ChInteractiveDriverIRR> irr_driver) { this->irr_driver = irr_driver; }

    std::shared_ptr<ChInteractiveDriverIRR> irr_driver;
};

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    LogCopyright(node_id == 0);

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // --------------
    // Create systems
    // --------------

    // Adjust position of each vehicle so they aren't on top of each other
    initLoc.y() = node_id * 3;

    // Get the vehicle JSON filenames
    double cam_distance;
    std::string vehicle_filename, engine_filename, transmission_filename, tire_filename, zombie_filename;
    GetVehicleModelFiles((VehicleType)cli.GetAsType<int>("vehicle"), vehicle_filename, engine_filename,
                         transmission_filename, tire_filename, zombie_filename, cam_distance);

    // Create the vehicle, set parameters, and initialize
    WheeledVehicle vehicle(vehicle_filename, contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(engine_filename);
    auto transmission = ReadTransmissionJSON(transmission_filename);
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(tire_filename);
            vehicle.InitializeTire(tire, wheel, tire_vis_type);
        }
    }

    // Set associated collision detection system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Add vehicle as an agent and initialize SynChronoManager
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, zombie_filename);
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(vehicle.GetSystem());

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile("terrain/RigidPlane.json"));

    // Create the vehicle Irrlicht interface
    IrrAppWrapper app;
    DriverWrapper driver(vehicle);
    if (cli.HasValueInVector<int>("irr", node_id)) {
        auto temp_app = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        temp_app->SetWindowTitle("SynChrono Wheeled Vehicle Demo");
        temp_app->SetChaseCamera(trackPoint, cam_distance, 0.5);
        temp_app->Initialize();
        temp_app->AddTypicalLights();
        temp_app->AttachVehicle(&vehicle);

        // Create the interactive driver system
        auto irr_driver = chrono_types::make_shared<ChInteractiveDriverIRR>(*temp_app);

        // Set the time response for steering and throttle keyboard inputs.
        double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        irr_driver->SetSteeringDelta(render_step_size / steering_time);
        irr_driver->SetThrottleDelta(render_step_size / throttle_time);
        irr_driver->SetBrakingDelta(render_step_size / braking_time);

        irr_driver->Initialize();

        app.Set(temp_app);
        driver.Set(irr_driver);
    }

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    while (app.IsOk() && syn_manager.IsOk()) {
        double time = vehicle.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time)
            break;

        // Render scene
        if (step_number % render_steps == 0)
            app.Render();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Log clock time
        if (step_number % 100 == 0 && node_id == 0) {
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            SynLog() << (time_span.count() / 1e3) / time << "\n";
        }
    }
    // Properly shuts down other ranks when one rank ends early
    syn_manager.QuitSimulation();

    return 0;
}

void LogCopyright(bool show) {
    if (!show)
        return;

    SynLog() << "Copyright (c) 2020 projectchrono.org\n";
    SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Nodes for irrlicht usage", "-1");

    // Other options
    cli.AddOption<int>("Demo", "v,vehicle", "Vehicle Options [0-4]: Sedan, HMMWV, UAZ, CityBus, MAN", "0");
}

void GetVehicleModelFiles(VehicleType type,
                          std::string& vehicle,
                          std::string& engine,
                          std::string& transmission,
                          std::string& tire,
                          std::string& zombie,
                          double& cam_distance) {
    switch (type) {
        case VehicleType::SEDAN:
            vehicle = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
            engine = vehicle::GetDataFile("sedan/powertrain/Sedan_EngineSimpleMap.json");
            transmission = vehicle::GetDataFile("sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json");
            tire = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
            zombie = synchrono::GetDataFile("vehicle/Sedan.json");
            cam_distance = 6.0;
            break;
        case VehicleType::HMMWV:
            vehicle = vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
            engine = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json");
            transmission = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json");
            tire = vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");
            zombie = synchrono::GetDataFile("vehicle/HMMWV.json");
            cam_distance = 6.0;
            break;
        case VehicleType::UAZ:
            vehicle = vehicle::GetDataFile("uaz/vehicle/UAZBUS_SAEVehicle.json");
            engine = vehicle::GetDataFile("uaz/powertrain/UAZBUS_EngineSimpleMap.json");
            transmission = vehicle::GetDataFile("uaz/powertrain/UAZBUS_AutomaticTransmissioniSimpleMap.json");
            tire = vehicle::GetDataFile("uaz/tire/UAZBUS_TMeasyTireFront.json");
            zombie = synchrono::GetDataFile("vehicle/UAZBUS.json");
            cam_distance = 6.0;
            break;
        case VehicleType::CITYBUS:
            vehicle = vehicle::GetDataFile("citybus/vehicle/CityBus_Vehicle.json");
            engine = vehicle::GetDataFile("citybus/powertrain/CityBus_EngineSimpleMap.json");
            transmission = vehicle::GetDataFile("citybus/powertrain/CityBus_AutomaticTransmissionSimpleMap.json");
            tire = vehicle::GetDataFile("citybus/tire/CityBus_TMeasyTire.json");
            zombie = synchrono::GetDataFile("vehicle/CityBus.json");
            cam_distance = 14.0;
            break;
        case VehicleType::MAN:
            vehicle = vehicle::GetDataFile("MAN_Kat1/vehicle/MAN_10t_Vehicle_8WD.json");
            engine = vehicle::GetDataFile("MAN_Kat1/powertrain/MAN_7t_EngineSimpleMap.json");
            transmission = vehicle::GetDataFile("MAN_Kat1/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.json");
            tire = vehicle::GetDataFile("MAN_Kat1/tire/MAN_5t_TMeasyTire.json");
            zombie = synchrono::GetDataFile("vehicle/MAN_8WD.json");
            cam_distance = 12.0;
            break;
    }
}