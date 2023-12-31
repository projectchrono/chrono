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
// Demo code for co-simulating a Chrono wheeld vehicle FMU and a path-follower
// driver FMU.
//
// =============================================================================

#include "chrono_vehicle/ChConfigVehicleFMI.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fmi/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------

std::string VEHICLE_FMU_MODEL_IDENTIFIER = "FMU_WheeledVehicle";
std::string DRIVER_FMU_MODEL_IDENTIFIER = "FMU_PathFollowerDriver";

std::string VEHICLE_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string("/");
std::string DRIVER_FMU_DIR = CHRONO_VEHICLE_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string("/");

std::string VEHICLE_FMU_FILENAME = VEHICLE_FMU_DIR + VEHICLE_FMU_MODEL_IDENTIFIER + std::string(".fmu");
std::string DRIVER_FMU_FILENAME = DRIVER_FMU_DIR + DRIVER_FMU_MODEL_IDENTIFIER + std::string(".fmu");

std::string VEHICLE_UNPACK_DIR = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_") + VEHICLE_FMU_MODEL_IDENTIFIER;
std::string DRIVER_UNPACK_DIR = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_") + DRIVER_FMU_MODEL_IDENTIFIER;

// -----------------------------------------------------------------------------

void CreateVehicleFMU(FmuChronoUnit& vehicle_fmu,
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
    std::cout << "Vehicle FMU version:  " << vehicle_fmu.GetVersion() << "\n";
    std::cout << "Vehicle FMU platform: " << vehicle_fmu.GetTypesPlatform() << "\n";

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
    ChVector<> init_loc(0, 0, 0.5);
    double init_yaw = 0;

    vehicle_fmu.SetVariable("vehicle_JSON", vehicle_JSON);
    vehicle_fmu.SetVariable("engine_JSON", engine_JSON);
    vehicle_fmu.SetVariable("transmission_JSON", transmission_JSON);
    vehicle_fmu.SetVecVariable("init_loc", init_loc);
    vehicle_fmu.SetVariable("init_yaw", init_yaw, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

void CreateDriverFMU(FmuChronoUnit& driver_fmu,
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
    std::string path_file = vehicle::GetDataFile("paths/ISO_double_lane_change2.txt");
    double throttle_threshold = 0.2;
    driver_fmu.SetVariable("path_file", path_file);
    driver_fmu.SetVariable("throttle_threshold", throttle_threshold, FmuVariable::Type::Real);

    //// TODO
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "./DEMO_WHEELEDVEHICLE_FMI_COSIM";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 20;

    // Create the 2 FMUs
    std::cout << "Vehicle FMU dir: >" << VEHICLE_FMU_DIR << "<" << std::endl;
    std::cout << "Driver FMU dir:  >" << DRIVER_FMU_DIR << "<" << std::endl;
    std::cout << "Vehicle FMU filename: >" << VEHICLE_FMU_FILENAME << "<" << std::endl;
    std::cout << "Driver FMU filename:  >" << DRIVER_FMU_FILENAME << "<" << std::endl;
    std::cout << "Vehicle FMU unpack directory: >" << VEHICLE_UNPACK_DIR << "<" << std::endl;
    std::cout << "Driver FMU unpack directory:  >" << DRIVER_UNPACK_DIR << "<" << std::endl;

    FmuChronoUnit vehicle_fmu;
    FmuChronoUnit driver_fmu;
    try {
        CreateVehicleFMU(vehicle_fmu, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return 1;
    }
    try {
        CreateDriverFMU(driver_fmu, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading driver FMU: " << e.what() << "\n";
        return 1;
    }

    // Initialize FMUs
    vehicle_fmu.EnterInitializationMode();
    driver_fmu.EnterInitializationMode();
    {
        ////ChVector<> g_acc(0, 0, 0);
        ////vehicle_fmu.SetVecVariable("g_acc", g_acc);

        // Optionally, enable run-time visualization for the vehicle and driver FMUs
        vehicle_fmu.SetVariable("vis", true, FmuVariable::Type::Boolean);
        driver_fmu.SetVariable("vis", true, FmuVariable::Type::Boolean);
    }
    vehicle_fmu.ExitInitializationMode();
    driver_fmu.ExitInitializationMode();

    // Simulation loop
    double time = 0;
    double dt = 1e-3;

    while (time < stop_time) {
        std::cout << "time = " << time << std::endl;

        // --------- Exchange data
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

        double target_speed = 10;
        driver_fmu.SetVariable("target_speed", target_speed, FmuVariable::Type::Real);

        // ----------- Advance FMUs
        vehicle_fmu.DoStep(time, dt, fmi2True);
        std::cout << "   vehicle advance done" << std::endl;
        driver_fmu.DoStep(time, dt, fmi2True);
        std::cout << "   driver advance done" << std::endl;

        time += dt;
    }

    return 0;
}
