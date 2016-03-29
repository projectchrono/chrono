// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Black-box program for using an external optimization program for tuning
// parameters of a PID steering controller.
//
// =============================================================================

#include <vector>
#include <valarray>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChCLineBezier.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace geometry;

// =============================================================================
// Global definitions

typedef std::valarray<double> DataArray;

struct Data {
    Data(int n) {
        time.resize(n);
        err_x.resize(n);
        err_y.resize(n);
        err_z.resize(n);
        err_speed.resize(n);
    }

    DataArray time;       // current time
    DataArray err_x;      // x component of vehicle location error
    DataArray err_y;      // y component of vehicle location error
    DataArray err_z;      // z component of vehicle location error
    DataArray err_speed;  // vehicle speed error
};

// Type of tire model
TireModelType tire_model = RIGID;

// Input file names for the path-follower driver model
std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file("generic/driver/SpeedController.json");
std::string path_file("paths/curve.txt");

// Output file name
std::string out_file("results.out");

// JSON file names for vehicle model, tire models, (simple) powertrain, and (rigid) terrain
std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones.json");
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string lugretire_file("generic/tire/LugreTire.json");
std::string fialatire_file("generic/tire/FialaTire.json");
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");
std::string rigidterrain_file("terrain/RigidPlane.json");

// Initial vehicle position and orientation
ChVector<> initLoc(-125, -125, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
double target_speed = 10;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Simulation step size and simulation length
double step_size = 2e-3;        // integration step size
int num_steps_settling = 3000;  // number of steps for settling
int num_steps = 5000;           // number of steps for data colection

// =============================================================================
// Forward declarations

void processData(const utils::CSV_writer& csv, const Data& data);

// =============================================================================
// Main driver program

int main(int argc, char* argv[]) {
    // Create and initialize the vehicle system
    WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file));
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));

    // Create and initialize the powertrain system
    SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
    powertrain.Initialize(vehicle.GetChassis(), vehicle.GetDriveshaft());

    // Create and initialize the tires
    int num_axles = vehicle.GetNumberAxles();
    int num_wheels = 2 * num_axles;

    std::vector<std::shared_ptr<ChTire> > tires(num_wheels);
    for (int i = 0; i < num_wheels; i++) {
        switch (tire_model) {
            case RIGID:
                tires[i] = std::make_shared<RigidTire>(vehicle::GetDataFile(rigidtire_file));
                break;
            case LUGRE:
                tires[i] = std::make_shared<LugreTire>(vehicle::GetDataFile(lugretire_file));
                break;
            case FIALA:
                tires[i] = std::make_shared<FialaTire>(vehicle::GetDataFile(fialatire_file));
                break;
        }
        tires[i]->Initialize(vehicle.GetWheelBody(i), VehicleSide(i % 2));
    }

    // Create the driver system
    ChBezierCurve* path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
    ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
                                vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);

    // Create a path tracker to keep track of the error in vehicle location.
    ChBezierCurveTracker tracker(path);

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize data collectors
    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    Data data(num_steps);

    // Inter-module communication data
    TireForces tire_forces(num_wheels);
    WheelStates wheel_states(num_wheels);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    for (int it = 0; it < num_steps_settling + num_steps; it++) {
        bool settling = (it < num_steps_settling);

        // Collect data
        if (!settling) {
            const ChVector<> sentinel = driver.GetSteeringController().GetSentinelLocation();
            const ChVector<> target = driver.GetSteeringController().GetTargetLocation();
            const ChVector<> vehicle_location = vehicle.GetChassisPos();
            ChVector<> vehicle_target;
            tracker.calcClosestPoint(vehicle_location, vehicle_target);
            ChVector<> vehicle_err = vehicle_target - vehicle_location;
            float speed_err = target_speed - vehicle.GetVehicleSpeed();

            csv << vehicle.GetChTime() << vehicle_location << vehicle_target << vehicle_err << speed_err << std::endl;

            int id = it - num_steps_settling;
            data.time[id] = vehicle.GetChTime();
            data.err_x[id] = vehicle_err.x;
            data.err_y[id] = vehicle_err.y;
            data.err_z[id] = vehicle_err.z;
            data.err_speed = speed_err;
        }

        // Collect output data from modules (for inter-module communication)
        if (settling) {
            throttle_input = 0;
            steering_input = 0;
            braking_input = 0;
        } else {
            throttle_input = driver.GetThrottle();
            steering_input = driver.GetSteering();
            braking_input = driver.GetBraking();
        }
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
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
    }

    processData(csv, data);

    return 0;
}

// =============================================================================
// Simulation data post-processing

void processData(const utils::CSV_writer& csv, const Data& data) {
    // Optionally, write simulation results to file for external post-processing
    csv.write_to_file(out_file);

    // Alternatively, post-process simulation results here and write out results
    DataArray loc_err_norm2 = data.err_x * data.err_x + data.err_y * data.err_y + data.err_z * data.err_z;
    double loc_L2_norm = std::sqrt(loc_err_norm2.sum());
    double loc_RMS_norm = std::sqrt(loc_err_norm2.sum() / num_steps);
    double loc_INF_norm = std::sqrt(loc_err_norm2.max());

    std::cout << "|location err|_L2 =  " << loc_L2_norm << std::endl;
    std::cout << "|location err|_RMS = " << loc_RMS_norm << std::endl;
    std::cout << "|location err|_INF = " << loc_INF_norm << std::endl;

    ////std::ofstream ofile(out_file.c_str());
    ////ofile << loc_L2_norm << std::endl;
    ////ofile.close();

    double speed_L2_norm = std::sqrt((data.err_speed * data.err_speed).sum());
    double speed_RMS_norm = std::sqrt((data.err_speed * data.err_speed).sum() / num_steps);
    double speed_INF_norm = std::abs(data.err_speed).max();

    std::cout << "|speed err|_L2 =  " << speed_L2_norm << std::endl;
    std::cout << "|speed err|_RMS = " << speed_RMS_norm << std::endl;
    std::cout << "|speed err|_INF = " << speed_INF_norm << std::endl;

    ////std::ofstream ofile(out_file.c_str());
    ////ofile << speed_L2_norm << std::endl;
    ////ofile.close();
}
