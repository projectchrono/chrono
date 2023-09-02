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
#include <future>
#include <chrono>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

using namespace chrono;
#ifdef CHRONO_IRRLICHT
using namespace chrono::irrlicht;
#endif
using namespace chrono::synchrono;
using namespace chrono::vehicle;

#undef ALIVE

// Quality of Service
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

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

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// USe IPv6
bool IPv6 = false;  // 100[Hz]

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);
void GetVehicleModelFiles(VehicleType type,
                          std::string& vehicle,
                          std::string& powertrain,
                          std::string& tire,
                          std::string& zombie,
                          double& cam_distance);
#ifdef CHRONO_IRRLICHT
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
#endif
// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, false, false))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");
    IPv6 = cli.GetAsType<bool>("ipv6");

    const int node_id = cli.GetAsType<int>("node_id");
    const int num_nodes = cli.GetAsType<int>("num_nodes");
    const std::vector<std::string> ip_list = cli.GetAsType<std::vector<std::string>>("ip");

    // Print help, if necessary
    if (cli.CheckHelp() && node_id == 1) {
        cli.Help();
        return 0;
    }
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    // Create the DomainParticipantQos
    DomainParticipantQos qos;
    qos.name("/syn/node/" + std::to_string(node_id) + ".0");
    if (IPv6) {
        // Use UDP6
        qos.transport().user_transports.push_back(std::make_shared<UDPv6TransportDescriptor>());

        qos.transport().use_builtin_transports = false;
        qos.wire_protocol().builtin.avoid_builtin_multicast = false;

        // Set the initialPeersList
        for (const auto& ip : ip_list) {
            Locator_t locator;
            locator.kind = LOCATOR_KIND_UDPv6;
            IPLocator::setIPv6(locator, ip);
            qos.wire_protocol().builtin.initialPeersList.push_back(locator);
        }
    } else {
        // Use UDP4
        qos.transport().user_transports.push_back(std::make_shared<UDPv4TransportDescriptor>());

        qos.transport().use_builtin_transports = false;
        qos.wire_protocol().builtin.avoid_builtin_multicast = false;

        // Set the initialPeersList
        for (const auto& ip : ip_list) {
            Locator_t locator;
            locator.kind = LOCATOR_KIND_UDPv4;
            IPLocator::setIPv4(locator, ip);
            qos.wire_protocol().builtin.initialPeersList.push_back(locator);
        }
    }
    auto communicator = chrono_types::make_shared<SynDDSCommunicator>(qos);
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // Copyright
    LogCopyright(node_id == 1);

    // --------------
    // Create systems
    // --------------

    // Adjust position of each vehicle so they aren't on top of each other
    initLoc.y() = node_id * 3;

    // Get the vehicle JSON filenames
    const std::string vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
    const std::string engine_filename = vehicle::GetDataFile("sedan/powertrain/Sedan_EngineSimpleMap.json");
    const std::string transmission_filename =
        vehicle::GetDataFile("sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json");
    const std::string tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
    const std::string zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");

    // Create the vehicle, set parameters, and initialize
    WheeledVehicle vehicle(vehicle_filename, contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON((engine_filename));
    auto transmission = ReadTransmissionJSON((transmission_filename));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(tire_filename);
            vehicle.InitializeTire(tire, wheel, tire_vis_type);
        }
    }

    // Add vehicle as an agent and initialize SynChronoManager
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, zombie_filename);
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(vehicle.GetSystem());

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile("terrain/RigidPlane.json"));

    // Create the vehicle Irrlicht interface
#ifdef CHRONO_IRRLICHT
    IrrAppWrapper app;
    DriverWrapper driver(vehicle);
    if (cli.HasValueInVector<int>("irr", node_id)) {
        auto temp_app = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        temp_app->SetWindowTitle("SynChrono Wheeled Vehicle Demo");
        temp_app->SetChaseCamera(trackPoint, 6.0, 0.5);
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
#else
    double target_speed = 10;
    // Calculate initial position and paths for each vehicle
    double pathLength = 1.5 * target_speed * end_time;

    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::shared_ptr<ChBezierCurve> path;
    if (node_id % 2 == 0) {
        // Start even vehicles in a row on the south side, driving north
        init_loc = ChVector<>(0, 2.0 * (node_id + 1), 0.5);
        init_rot = Q_from_AngZ(0);
        path = StraightLinePath(init_loc, init_loc + ChVector<>(pathLength, 0, 0));
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        init_loc = ChVector<>(2.0 * (node_id - 1), -5.0 - 2.0 * (node_id - 1), 0.5);
        init_rot = Q_from_AngZ(CH_C_PI / 2);
        path = StraightLinePath(init_loc, init_loc + ChVector<>(0, pathLength, 0));
    }
    ChPathFollowerDriver driver(vehicle, path, "Box path", target_speed);
    driver.Initialize();

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(2);
#endif
    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    bool appok = true;
    while (appok && syn_manager.IsOk()) {
        double time = vehicle.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time)
            break;

            // Render scene
#ifdef CHRONO_IRRLICHT
        if (step_number % render_steps == 0)
            app.Render();

#endif
        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        // std::async(&syn_manager.Synchronize, time);  // Synchronize between nodes
        std::async(std::launch::async, &SynChronoManager::Synchronize, &syn_manager, time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        driver.Synchronize(time);
#ifdef CHRONO_IRRLICHT
        app.Synchronize(time, driver_inputs);
#endif
        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        app.Advance(step_size);
#endif
        // Increment frame number
        step_number++;

        // Log clock time
        if (step_number % 100 == 0) {
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            SynLog() << (time_span.count() / 1e3) / time << "\n";
            std::cout << std::flush;
        }

#ifdef CHRONO_IRRLICHT
        appok = app.IsOk();
#else
        appok = true;
#endif
    }

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
    cli.AddOption<bool>("Demo", "p,ipv6", "Use IPv6", "false");

    // DDS Specific
    cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
    cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
    cli.AddOption<std::vector<std::string>>("DDS", "ip", "IP Addresses for initialPeersList", "127.0.0.1");
}
