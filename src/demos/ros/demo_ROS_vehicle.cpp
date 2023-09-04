#include "rclcpp/rclcpp.hpp"

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSDriverInputsHandler.h"

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/ChSensorManager.h"
    #include "chrono_sensor/sensors/ChCameraSensor.h"
    #include "chrono_sensor/sensors/ChLidarSensor.h"
    #include "chrono_sensor/sensors/ChIMUSensor.h"
    #include "chrono_sensor/sensors/ChGPSSensor.h"
    #include "chrono_sensor/filters/ChFilterAccess.h"

    #include "chrono_ros/handlers/ChROSCameraHandler.h"
    #include "chrono_ros/handlers/ChROSAccelerometerHandler.h"
    #include "chrono_ros/handlers/ChROSGyroscopeHandler.h"
    #include "chrono_ros/handlers/ChROSMagnetometerHandler.h"
    #include "chrono_ros/handlers/ChROSLidarHandler.h"
    #include "chrono_ros/handlers/ChROSGPSHandler.h"
#endif

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"

#include <chrono>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;

#ifdef CHRONO_SENSOR
using namespace chrono::sensor;
#endif

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
// auto vehicle_model = Polaris_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation
ChVector<> initLoc(0, 0, 0.5);
double initYaw = 20 * CH_C_DEG_TO_RAD;

// Simulation step size
double step_size = 2e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_model.VehicleJSON()), vehicle_model.ContactMethod());
    vehicle.Initialize(ChCoordsys<>(initLoc, Q_from_AngZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(vehicle_model.TireJSON()));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto system = vehicle.GetSystem();

    // Create the terrain
    RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the sensor system (if Chrono::Sensor is available)
#ifdef CHRONO_SENSOR
    auto manager = chrono_types::make_shared<ChSensorManager>(system);
    manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    // Create a camera that's placed on the hood
    auto cam_offset_pose({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassisBody(),  // body camera is attached to
                                                         30,                        // update rate in Hz
                                                         cam_offset_pose,           // offset pose
                                                         280,                       // image width
                                                         120,                       // image height
                                                         CH_C_PI / 4);              // camera's horizontal field of view
    cam->SetName("Camera Sensor");
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    // Create a lidar and add it to the sensor manager
    auto lidar_offset_pose = chrono::ChFrame<double>({-4, 0, 1}, Q_from_AngAxis(0, {0, 1, 0}));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(vehicle.GetChassisBody(),  // body lidar is attached to
                                                          5.f,                       // scanning rate in Hz
                                                          lidar_offset_pose,         // offset pose
                                                          900,                       // number of horizontal samples
                                                          30,                        // number of vertical channels
                                                          2 * CH_C_PI,               // horizontal field of view
                                                          CH_C_PI / 12, -CH_C_PI / 6, 100.0f  // vertical field of view
    );
    lidar->SetName("Lidar Sensor");
    acc->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());  // Add a filter to access the point cloud data
    manager->AddSensor(lidar);

    // add an accelerometer, gyroscope, and magnetometer
    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChVector<> gps_reference(-89.400, 43.070, 260.0);
    auto acc =
        chrono_types::make_shared<ChAccelerometerSensor>(vehicle.GetChassisBody(),  // body to which the IMU is attached
                                                         100.f,                     // update rate
                                                         imu_offset_pose,           // offset pose from body
                                                         noise_none);               // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro =
        chrono_types::make_shared<ChGyroscopeSensor>(vehicle.GetChassisBody(),  // body to which the IMU is attached
                                                     100.f,                     // update rate
                                                     imu_offset_pose,           // offset pose from body
                                                     noise_none);               // IMU noise model
    gyro->SetName("IMU - Gyroscope");
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(vehicle.GetChassisBody(),  // body to which the IMU is attached
                                                        100.f,                     // update rate
                                                        imu_offset_pose,           // offset pose from body
                                                        noise_none,                // IMU noise model
                                                        gps_reference);
    mag->SetName("IMU - Magnetometer");
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());  // Add a filter to access the imu data
    manager->AddSensor(mag);                                             // Add the IMU sensor to the sensor manager

    // add a GPS sensor
    auto gps_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto gps = chrono_types::make_shared<ChGPSSensor>(
        vehicle.GetChassisBody(),  // body to which the GPS is attached
        5.f,                       // update rate
        gps_offset_pose,           // offset pose from body
        gps_reference,             // reference GPS location (GPS coordinates of simulation origin)
        noise_none                 // noise model to use for adding GPS noise
    );
    gps->SetName("GPS");
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps);                                          // Add GPS sensor to the sensor manager
#endif

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle);

    // Create ROS manager
    auto manager = chrono_types::make_shared<ChROSManager>();
    manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());
    manager->RegisterHandler(25, chrono_types::make_shared<ChROSDriverInputsHandler>(driver));
#ifdef CHRONO_SENSOR
    manager->RegisterHandler(cam->GetUpdateRate(), chrono_types::make_shared<ChROSCameraHandler>(cam));
    manager->RegisterHandler(lidar->GetUpdateRate(), chrono_types::make_shared<ChROSLidarHandler>(lidar));
    manager->RegisterHandler(acc->GetUpdateRate(), chrono_types::make_shared<ChROSAccelerometerHandler>(acc));
    manager->RegisterHandler(gyro->GetUpdateRate(), chrono_types::make_shared<ChROSGyroscopeHandler>(gyro));
    manager->RegisterHandler(mag->GetUpdateRate(), chrono_types::make_shared<ChROSMagnetometerHandler>(mag));
    manager->RegisterHandler(gps->GetUpdateRate(), chrono_types::make_shared<ChROSGPSHandler>(gps));
#endif
    manager->Initialize();

    // Simulation loop
    vehicle.EnableRealtime(true);
    while (rclcpp::ok()) {
        // Get driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        manager->Update(step_size);
    }

    return 0;
}
