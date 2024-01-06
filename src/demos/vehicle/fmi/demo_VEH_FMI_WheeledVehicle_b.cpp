// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo illkustrating the co-simulation of a Chrono wheeled vehicle FMU, 4 tire
// FMUs, and a path-follower driver FMU.
//
// =============================================================================

#include <array>

#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fmi/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

std::string VEHICLE_FMU_MODEL_IDENTIFIER = "FMU_WheeledVehicle";
std::string TIRE_FMU_MODEL_IDENTIFIER = "FMU_ForceElementTire";
std::string DRIVER_FMU_MODEL_IDENTIFIER = "FMU_PathFollowerDriver";

std::string VEHICLE_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string TIRE_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + TIRE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string DRIVER_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string("/");

std::string VEHICLE_FMU_FILENAME = VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string(".fmu");
std::string TIRE_FMU_FILENAME = TIRE_FMU_DIR + TIRE_FMU_MODEL_IDENTIFIER + std::string(".fmu");
std::string DRIVER_FMU_FILENAME = DRIVER_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string(".fmu");

std::string VEHICLE_UNPACK_DIR = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_") + VEHICLE_FMU_MODEL_IDENTIFIER;
std::string TIRE_UNPACK_DIR = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_") + TIRE_FMU_MODEL_IDENTIFIER;
std::string DRIVER_UNPACK_DIR = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_") + DRIVER_FMU_MODEL_IDENTIFIER;

// -----------------------------------------------------------------------------

void CreateVehicleFMU(FmuChronoUnit& vehicle_fmu,
                      double step_size,
                      double start_time,
                      double stop_time,
                      const std::vector<std::string>& logCategories) {
    try {
        vehicle_fmu.Load(VEHICLE_FMU_FILENAME, VEHICLE_UNPACK_DIR);
        vehicle_fmu.BuildVariablesTree();
        vehicle_fmu.BuildVisualizersList(&vehicle_fmu.tree_variables);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Vehicle FMU version:  " << vehicle_fmu.GetVersion() << std::endl;
    std::cout << "Vehicle FMU platform: " << vehicle_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    vehicle_fmu.Instantiate("WheeledVehicleFmuComponent");

    // Set debug logging
    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    vehicle_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                start_time,             // start time
                                fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    std::string vehicle_JSON = vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");
    std::string engine_JSON = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json");
    std::string transmission_JSON = vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json");

    vehicle_fmu.SetVariable("vehicle_JSON", vehicle_JSON);
    vehicle_fmu.SetVariable("engine_JSON", engine_JSON);
    vehicle_fmu.SetVariable("transmission_JSON", transmission_JSON);
    vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);

    ////ChVector<> g_acc(0, 0, 0);
    ////vehicle_fmu.SetVecVariable("g_acc", g_acc);
}

// -----------------------------------------------------------------------------

void CreateDriverFMU(FmuChronoUnit& driver_fmu,
                     double step_size,
                     double start_time,
                     double stop_time,
                     const std::vector<std::string>& logCategories) {
    try {
        driver_fmu.Load(DRIVER_FMU_FILENAME, DRIVER_UNPACK_DIR);
        driver_fmu.BuildVariablesTree();
        driver_fmu.BuildVisualizersList(&driver_fmu.tree_variables);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Driver FMU version:  " << driver_fmu.GetVersion() << "\n";
    std::cout << "Driver FMU platform: " << driver_fmu.GetTypesPlatform() << "\n";

    // Instantiate FMU
    driver_fmu.Instantiate("DriverFmuComponent");

    // Set debug logging
    driver_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    driver_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                               start_time,             // start time
                               fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    std::string path_file = vehicle::GetDataFile("paths/ISO_double_lane_change.txt");
    ////std::string path_file = vehicle::GetDataFile("paths/ISO_double_lane_change2.txt");
    double throttle_threshold = 0.2;
    double look_ahead_dist = 5.0;

    driver_fmu.SetVariable("path_file", path_file);
    driver_fmu.SetVariable("throttle_threshold", throttle_threshold, FmuVariable::Type::Real);
    driver_fmu.SetVariable("look_ahead_dist", look_ahead_dist, FmuVariable::Type::Real);
    driver_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

void CreateTireFMU(FmuChronoUnit& tire_fmu,
                   double step_size,
                   double start_time,
                   double stop_time,
                   const std::vector<std::string>& logCategories) {
    try {
        tire_fmu.Load(TIRE_FMU_FILENAME, TIRE_UNPACK_DIR);
        tire_fmu.BuildVariablesTree();
        tire_fmu.BuildVisualizersList(&tire_fmu.tree_variables);
    } catch (std::exception& e) {
        throw e;
    }
    std::cout << "Tire FMU version:  " << tire_fmu.GetVersion() << std::endl;
    std::cout << "Tire FMU platform: " << tire_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    tire_fmu.Instantiate("ForceElementTireFmuComponent");

    // Set debug logging
    tire_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    tire_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                             start_time,             // start time
                             fmi2False, stop_time);  // use stop time

    // Set fixed parameters
    std::string tire_JSON = vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");

    tire_fmu.SetVariable("tire_JSON", tire_JSON);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // -----------------------
    // Create output directory
    // -----------------------

    std::string out_dir = GetChronoOutputPath() + "./DEMO_WHEELEDVEHICLE_FMI_COSIM_B";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 20;
    double step_size = 1e-3;

    bool vis_vehicle = true;
    bool vis_driver = true;

    // -----------------------
    // Create the FMUs
    // -----------------------

    ////std::cout << "Vehicle FMU dir: >" << VEHICLE_FMU_DIR << "<" << std::endl;
    ////std::cout << "Tire FMU dir:    >" << TIRE_FMU_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU dir:  >" << DRIVER_FMU_DIR << "<" << std::endl;
    ////std::cout << "Vehicle FMU filename: >" << VEHICLE_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Tire FMU filename:    >" << TIRE_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Driver FMU filename:  >" << DRIVER_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Vehicle FMU unpack directory: >" << VEHICLE_UNPACK_DIR << "<" << std::endl;
    ////std::cout << "Tire FMU unpack directory:    >" << TIRE_UNPACK_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU unpack directory:  >" << DRIVER_UNPACK_DIR << "<" << std::endl;

    FmuChronoUnit vehicle_fmu;
    try {
        CreateVehicleFMU(vehicle_fmu, step_size, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return 1;
    }

    FmuChronoUnit driver_fmu;
    try {
        CreateDriverFMU(driver_fmu, step_size, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading driver FMU: " << e.what() << "\n";
        return 1;
    }

    FmuChronoUnit tire_fmu[4];
    for (int i = 0; i < 4; i++) {
        try {
            CreateTireFMU(tire_fmu[i], step_size, start_time, stop_time, logCategories);
        } catch (std::exception& e) {
            std::cout << "ERROR loading tire FMU: " << e.what() << "\n";
            return 1;
        }
    }

    // -----------------------
    // Initialize FMUs
    // -----------------------

    driver_fmu.EnterInitializationMode();
    {
        // Optionally, enable run-time visualization for the driver FMU
        driver_fmu.SetVariable("vis", vis_driver);
    }
    driver_fmu.ExitInitializationMode();

    vehicle_fmu.EnterInitializationMode();
    {
        // Set initial vehicle location
        ChVector<> init_loc;
        double init_yaw;
        driver_fmu.GetVecVariable("init_loc", init_loc);
        driver_fmu.GetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
        vehicle_fmu.SetVecVariable("init_loc", init_loc);
        vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);

        // Optionally, enable run-time visualization for the vehicle FMU
        vehicle_fmu.SetVariable("vis", vis_vehicle);
    }
    vehicle_fmu.ExitInitializationMode();

    for (int i = 0; i < 4; i++) {
        tire_fmu[i].EnterInitializationMode();
        tire_fmu[i].ExitInitializationMode();
    }

    // -----------------------
    // Create terrain
    // -----------------------

    FlatTerrain terrain(0.0, 0.8f);

    // -----------------------
    // Simulation loop
    // -----------------------

    std::string wheel_id[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

    double time = 0;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        ////std::cout << "time = " << time << std::endl;

        // ----------- Set FMU control variables
        double target_speed = 12;
        driver_fmu.SetVariable("target_speed", target_speed, FmuVariable::Type::Real);

        // --------- Exchange data between vehicle and driver FMUs
        double steering;
        double throttle;
        double braking;
        driver_fmu.GetVariable("steering", steering, FmuVariable::Type::Real);
        driver_fmu.GetVariable("throttle", throttle, FmuVariable::Type::Real);
        driver_fmu.GetVariable("braking", braking, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("steering", steering, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("throttle", throttle, FmuVariable::Type::Real);
        vehicle_fmu.SetVariable("braking", braking, FmuVariable::Type::Real);

        ChFrameMoving<> ref_frame;
        vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);
        driver_fmu.SetFrameMovingVariable("ref_frame", ref_frame);

        // ----------- Exchange data between vehicle and tire FMUs
        for (int i = 0; i < 4; i++) {
            WheelState state;
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".pos", state.pos);
            vehicle_fmu.GetQuatVariable(wheel_id[i] + ".rot", state.rot);
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".lin_vel", state.lin_vel);
            vehicle_fmu.GetVecVariable(wheel_id[i] + ".ang_vel", state.ang_vel);
            tire_fmu[i].SetVecVariable("wheel_state.pos", state.pos);
            tire_fmu[i].SetQuatVariable("wheel_state.rot", state.rot);
            tire_fmu[i].SetVecVariable("wheel_state.lin_vel", state.lin_vel);
            tire_fmu[i].SetVecVariable("wheel_state.ang_vel", state.ang_vel);

            TerrainForce load;
            tire_fmu[i].GetVecVariable("wheel_load.point", load.point);
            tire_fmu[i].GetVecVariable("wheel_load.force", load.force);
            tire_fmu[i].GetVecVariable("wheel_load.moment", load.moment);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".point", load.point);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".force", load.force);
            vehicle_fmu.SetVecVariable(wheel_id[i] + ".moment", load.moment);
        }

        // ------------ Set terrain information for each tire FMU
        for (int i = 0; i < 4; i++) {
            ChVector<> query_point;
            tire_fmu[i].GetVecVariable("query_point", query_point);

            double terrain_height = terrain.GetHeight(query_point);
            ChVector<> terrain_normal = terrain.GetNormal(query_point);
            double terrain_mu = (double)terrain.GetCoefficientFriction(query_point);
            tire_fmu[i].SetVariable("terrain_height", terrain_height, FmuVariable::Type::Real);
            tire_fmu[i].SetVecVariable("terrain_normal", terrain_normal);
            tire_fmu[i].SetVariable("terrain_mu", terrain_mu, FmuVariable::Type::Real);
        }

        // ----------- Advance FMUs
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        auto status_driver = driver_fmu.DoStep(time, step_size, fmi2True);
        if (status_vehicle == fmi2Discard || status_driver == fmi2Discard)
            break;
        for (int i = 0; i < 4; i++) {
            auto status_tire = tire_fmu[i].DoStep(time, step_size, fmi2True);
            if (status_tire == fmi2Discard)
                break;
        }

        time += step_size;
    }

    timer.stop();
    std::cout << "Sim time: " << time << std::endl;
    std::cout << "Run time: " << timer() << std::endl;

    return 0;
}
