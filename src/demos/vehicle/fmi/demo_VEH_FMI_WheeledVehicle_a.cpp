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
// Demo illustrating the co-simulation of a Chrono wheeled vehicle FMU and a
// path-follower driver FMU.
//
// =============================================================================

#include <array>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

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

class DummyWheel : public ChWheel {
  public:
    DummyWheel() : ChWheel("tire_wheel"), m_inertia(ChVector<>(0)) {}
    virtual double GetWheelMass() const override { return 0; }
    virtual const ChVector<>& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }

  private:
    ChVector<> m_inertia;
};

struct WheelTire {
    std::string id;
    std::shared_ptr<ChWheel> wheel;
    std::shared_ptr<ChTire> tire;
};

void CreateTires(ChSystem& sys, std::array<WheelTire, 4>& wt) {
    std::string tire_JSON = vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json");

    std::string id[4] = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

    for (int i = 0; i < 4; i++) {
        wt[i].id = id[i];
        auto spindle = chrono_types::make_shared<ChBody>();
        sys.AddBody(spindle);

        wt[i].wheel = chrono_types::make_shared<DummyWheel>();
        wt[i].wheel->Initialize(nullptr, spindle, LEFT);

        wt[i].tire = ReadTireJSON(tire_JSON);
        wt[i].wheel->SetTire(wt[i].tire);
        wt[i].tire->Initialize(wt[i].wheel);
    }
}

void SynchronizeTires(double time, FmuChronoUnit& vehicle_fmu, ChTerrain& terrain, std::array<WheelTire, 4>& wt) {
    for (int i = 0; i < 4; i++) {
        // Get wheel state from vehicle FMU
        WheelState state;
        vehicle_fmu.GetVecVariable(wt[i].id + ".pos", state.pos);
        vehicle_fmu.GetQuatVariable(wt[i].id + ".rot", state.rot);
        vehicle_fmu.GetVecVariable(wt[i].id + ".lin_vel", state.lin_vel);
        vehicle_fmu.GetVecVariable(wt[i].id + ".ang_vel", state.ang_vel);

        // Get tire force
        auto force = wt[i].tire->ReportTireForce(&terrain);

        // Set spindle/wheel state and synchronize tire
        auto spindle = wt[i].wheel->GetSpindle();
        spindle->SetPos(state.pos);
        spindle->SetRot(state.rot);
        spindle->SetPos_dt(state.lin_vel);
        spindle->SetWvel_par(state.ang_vel);
        wt[i].tire->Synchronize(time, terrain);

        // Set tire force to vehicle FMU
        vehicle_fmu.SetVecVariable(wt[i].id + ".point", force.point);
        vehicle_fmu.SetVecVariable(wt[i].id + ".force", force.force);
        vehicle_fmu.SetVecVariable(wt[i].id + ".moment", force.moment);

        ////if (i == 3) {
        ////    std::cout << "   state.pos     = " << state.pos << std::endl;
        ////    std::cout << "   state.rot     = " << state.rot << std::endl;
        ////    std::cout << "   state.lin_vel = " << state.lin_vel << std::endl;
        ////    std::cout << "   state.ang_vel = " << state.ang_vel << std::endl;
        ////    std::cout << std::endl;
        ////    std::cout << "   tire force    = " << force.force << std::endl;
        ////    std::cout << "   tire torque   = " << force.moment << std::endl;
        ////    std::cout << std::endl;
        ////}
    }
}

void AdvanceTires(double step_size, std::array<WheelTire, 4>& wt) {
    for (int i = 0; i < 4; i++) {
        wt[i].tire->Advance(step_size);
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create (if needed) output directory
    std::string out_dir = GetChronoOutputPath() + "./DEMO_WHEELEDVEHICLE_FMI_COSIM_A";

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

    // Create the 2 FMUs
    ////std::cout << "Vehicle FMU dir: >" << VEHICLE_FMU_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU dir:  >" << DRIVER_FMU_DIR << "<" << std::endl;
    ////std::cout << "Vehicle FMU filename: >" << VEHICLE_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Driver FMU filename:  >" << DRIVER_FMU_FILENAME << "<" << std::endl;
    ////std::cout << "Vehicle FMU unpack directory: >" << VEHICLE_UNPACK_DIR << "<" << std::endl;
    ////std::cout << "Driver FMU unpack directory:  >" << DRIVER_UNPACK_DIR << "<" << std::endl;

    FmuChronoUnit vehicle_fmu;
    FmuChronoUnit driver_fmu;
    try {
        CreateVehicleFMU(vehicle_fmu, step_size, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return 1;
    }
    try {
        CreateDriverFMU(driver_fmu, step_size, start_time, stop_time, logCategories);
    } catch (std::exception& e) {
        std::cout << "ERROR loading driver FMU: " << e.what() << "\n";
        return 1;
    }

    // Initialize FMUs
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

    // Create terrain
    FlatTerrain terrain(0.0, 0.8f);

    // Create wheels and tires
    ChSystemSMC sys;
    std::array<WheelTire, 4> wt;
    CreateTires(sys, wt);

    // Simulation loop
    double time = 0;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        ////std::cout << "time = " << time << std::endl;

        // ----------- Set FMU control variables
        double target_speed = 12;
        driver_fmu.SetVariable("target_speed", target_speed, FmuVariable::Type::Real);

        // --------- Exchange data between FMUs
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

        // ----------- Exchange data between vehicle FMU and tires
        SynchronizeTires(time, vehicle_fmu, terrain, wt);

        // ----------- Advance FMUs
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        auto status_driver = driver_fmu.DoStep(time, step_size, fmi2True);

        if (status_vehicle == fmi2Discard || status_driver == fmi2Discard)
            break;

        // ----------- Advance system for tires
        AdvanceTires(step_size, wt);
        sys.DoStepDynamics(step_size);

        time += step_size;
    }

    timer.stop();
    std::cout << "Sim time: " << time << std::endl;
    std::cout << "Run time: " << timer() << std::endl;

    return 0;
}
