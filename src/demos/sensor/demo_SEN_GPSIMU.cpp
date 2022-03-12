// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Chrono demonstration of a GPS and an IMU sensor
// Attaches GPS and IMU sensors to a double pendulum system
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// IMU parameters
// -----------------------------------------------------------------------------
// Noise model attached to the sensor
enum IMUNoiseModel {
    NORMAL_DRIFT,  // gaussian drifting noise with noncorrelated equal distributions
    IMU_NONE       // no noise added
};
IMUNoiseModel imu_noise_type = NORMAL_DRIFT;

// IMU update rate in Hz
int imu_update_rate = 100;

// IMU lag (in seconds) between sensing and when data becomes accessible
float imu_lag = 0;

// IMU collection time (in seconds) of each sample
float imu_collection_time = 0;

// -----------------------------------------------------------------------------
// GPS parameters
// -----------------------------------------------------------------------------
// Noise model attached to the sensor
enum GPSNoiseModel {
    NORMAL,    // individually parameterized independent gaussian distribution
    GPS_NONE,  // no noise model
};
GPSNoiseModel gps_noise_type = GPS_NONE;

// GPS update rate in Hz
int gps_update_rate = 10;

// Camera's horizontal field of view
float fov = 1.408f;

// GPS lag (in seconds) between sensing and when data becomes accessible
float gps_lag = 0;

// Collection time (in seconds) of eacn sample
float gps_collection_time = 0;

// Origin used as the gps reference point
// Located in Madison, WI
ChVector<> gps_reference(-89.400, 43.070, 260.0);

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 20.0f;

// Save data
bool save = true;

// Output directories
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;
    sys.Set_G_acc({0, 0, -9.81});

    // -------------------------------
    // Create a double pendulum system
    // -------------------------------
    auto base = chrono_types::make_shared<ChBodyEasyBox>(.2, .2, 1, 1000, true, false);
    base->SetPos(ChVector<>(-.2, 0, .5));
    base->SetBodyFixed(true);  // the truss does not move!
    sys.Add(base);

    auto pendulum_leg_1 = chrono_types::make_shared<ChBodyEasyBox>(.2, .4, .2, 1000, true, false);
    pendulum_leg_1->SetPos(ChVector<>(0, .2, 1));
    pendulum_leg_1->SetBodyFixed(false);
    sys.Add(pendulum_leg_1);

    auto pendulum_leg_2 = chrono_types::make_shared<ChBodyEasyBox>(.2, .4, .2, 1000, true, false);
    pendulum_leg_2->SetPos(ChVector<>(0, .6, 1));
    pendulum_leg_2->SetBodyFixed(false);
    sys.Add(pendulum_leg_2);

    auto plate = chrono_types::make_shared<ChBodyEasyBox>(.4, .2, .2, 1000, true, false);
    plate->SetPos(ChVector<>(0, 0, 4));
    plate->SetBodyFixed(true);
    sys.Add(plate);

    auto link1 = chrono_types::make_shared<ChLinkLockRevolute>();
    link1->Initialize(base, pendulum_leg_1, ChCoordsys<>({0, 0, 1}, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    sys.AddLink(link1);

    auto link2 = chrono_types::make_shared<ChLinkLockRevolute>();
    link2->Initialize(pendulum_leg_1, pendulum_leg_2,
                      ChCoordsys<>({0, .4, 1}, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    sys.AddLink(link2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    // ---------------------------------------------
    // Create a IMU and add it to the sensor manager
    // ---------------------------------------------
    // Create the imu noise model
    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;
    switch (imu_noise_type) {
        case NORMAL_DRIFT:
            // Set the imu noise model to a gaussian model
            acc_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                          //
                                                              ChVector<double>({0., 0., 0.}),           // mean,
                                                              ChVector<double>({0.001, 0.001, 0.001}),  // stdev,
                                                              .0001,                                    // bias_drift,
                                                              .1);                                      // tau_drift,
            gyro_noise_model =
                chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                 // float updateRate,
                                                              ChVector<double>({0., 0., 0.}),  // float mean,
                                                              ChVector<double>({0.001, 0.001, 0.001}),  // float
                                                              .001,  // double bias_drift,
                                                              .1);   // double tau_drift,
            mag_noise_model =
                chrono_types::make_shared<ChNoiseNormal>(ChVector<double>({0., 0., 0.}),            // float mean,
                                                         ChVector<double>({0.001, 0.001, 0.001}));  // float stdev,
            break;
        case IMU_NONE:
            // Set the imu noise model to none (does not affect the data)
            acc_noise_model = chrono_types::make_shared<ChNoiseNone>();
            gyro_noise_model = chrono_types::make_shared<ChNoiseNone>();
            mag_noise_model = chrono_types::make_shared<ChNoiseNone>();
            break;
    }

    // add an accelerometer, gyroscope, and magnetometer to one of the pendulum legs
    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(pendulum_leg_1,    // body to which the IMU is attached
                                                                imu_update_rate,   // update rate
                                                                imu_offset_pose,   // offset pose from body
                                                                acc_noise_model);  // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(pendulum_leg_1,     // body to which the IMU is attached
                                                             imu_update_rate,    // update rate
                                                             imu_offset_pose,    // offset pose from body
                                                             gyro_noise_model);  // IMU noise model
    gyro->SetName("IMU - Accelerometer");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(plate,            // body to which the IMU is attached
                                                               imu_update_rate,  // update rate
                                                               imu_offset_pose,  // offset pose from body
                                                               mag_noise_model,  // IMU noise model
                                                               gps_reference);
    mag->SetName("IMU - Accelerometer");
    mag->SetLag(imu_lag);
    mag->SetCollectionWindow(imu_collection_time);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());  // Add a filter to access the imu data
    manager->AddSensor(mag);                                             // Add the IMU sensor to the sensor manager

    // ---------------------------------------------
    // Create a GPS and add it to the sensor manager
    // ---------------------------------------------
    // Create the gps noise model
    std::shared_ptr<ChNoiseModel> gps_noise_model;
    switch (gps_noise_type) {
        case NORMAL:
            // Set the gps noise model to a gaussian model
            gps_noise_model =
                chrono_types::make_shared<ChNoiseNormal>(ChVector<double>(1.f, 1.f, 1.f),  // Mean
                                                         ChVector<double>(2.f, 3.f, 1.f)   // Standard Deviation
                );
            break;
        case GPS_NONE:
            // Set the gps noise model to none (does not affect the data)
            gps_noise_model = chrono_types::make_shared<ChNoiseNone>();
            break;
    }

    // add a GPS sensor to one of the boxes
    auto gps_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto gps = chrono_types::make_shared<ChGPSSensor>(
        pendulum_leg_2,   // body to which the GPS is attached
        gps_update_rate,  // update rate
        gps_offset_pose,  // offset pose from body
        gps_reference,    // reference GPS location (GPS coordinates of simulation origin)
        gps_noise_model   // noise model to use for adding GPS noise
    );
    gps->SetName("GPS");
    gps->SetLag(gps_lag);
    gps->SetCollectionWindow(gps_collection_time);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());  // Add a filter to access the gps data
    manager->AddSensor(gps);                                          // Add GPS sensor to the sensor manager

    // -----------------
    // Initialize output
    // -----------------

    std::string imu_file = out_dir + "imu/";
    std::string gps_file = out_dir + "gps/";

    if (!filesystem::create_directory(filesystem::path(imu_file))) {
        std::cout << "Error creating directory " << imu_file << std::endl;
        return 1;
    }

    if (!filesystem::create_directory(filesystem::path(gps_file))) {
        std::cout << "Error creating directory " << gps_file << std::endl;
        return 1;
    }

    // Create a CSV writer to record the IMU data
    imu_file += "pendulum_leg_1.csv";
    utils::CSV_writer imu_csv(" ");

    // Create a CSV writer to record the GPS data
    gps_file += "pendulum_leg_2.csv";
    utils::CSV_writer gps_csv(" ");

    // ---------------
    // Simulate system
    // ---------------
    float ch_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    UserAccelBufferPtr bufferAcc;
    UserGyroBufferPtr bufferGyro;
    UserMagnetBufferPtr bufferMag;
    UserGPSBufferPtr bufferGPS;

    int imu_last_launch = 0;
    int gps_last_launch = 0;

    double rot_rate = 1;
    double ang;
    ChVector<double> axis;

    while (ch_time < end_time) {
        plate->SetRot(Q_from_AngZ(rot_rate * ch_time));
        // Get the most recent imu data
        bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();
        if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
            bufferMag->LaunchedCount > imu_last_launch) {
            // Save the imu data to file
            AccelData acc_data = bufferAcc->Buffer[0];
            GyroData gyro_data = bufferGyro->Buffer[0];
            MagnetData mag_data = bufferMag->Buffer[0];

            plate->GetRot().Q_to_AngAxis(ang, axis);

            imu_csv << std::fixed << std::setprecision(6);
            imu_csv << acc_data.X;
            imu_csv << acc_data.Y;
            imu_csv << acc_data.Z;
            imu_csv << gyro_data.Roll;
            imu_csv << gyro_data.Pitch;
            imu_csv << gyro_data.Yaw;
            imu_csv << mag_data.X;
            imu_csv << mag_data.Y;
            imu_csv << mag_data.Z;
            imu_csv << std::endl;
            imu_last_launch = bufferMag->LaunchedCount;
        }

        // Get the most recent gps data
        bufferGPS = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS->Buffer && bufferGPS->LaunchedCount > gps_last_launch) {
            // Save the gps data to file
            GPSData gps_data = bufferGPS->Buffer[0];
            gps_csv << std::fixed << std::setprecision(6);
            gps_csv << gps_data.Latitude;   // Latitude
            gps_csv << gps_data.Longitude;  // Longitude
            gps_csv << gps_data.Altitude;   // Altitude
            gps_csv << gps_data.Time;       // Time
            gps_csv << std::endl;
            gps_last_launch = bufferGPS->LaunchedCount;
        }

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    imu_csv.write_to_file(imu_file);
    gps_csv.write_to_file(gps_file);

    return 0;
}
