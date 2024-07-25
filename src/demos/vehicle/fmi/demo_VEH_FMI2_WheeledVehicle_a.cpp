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
// Demo illustrating the co-simulation of a Chrono wheeled vehicle FMU with
// external subsystems for terrain, tires, powertrain, and driver.
//
// The wheeled vehicle FMU used here is assumed to not include a powertrain model.
// =============================================================================

#include <array>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fmi/fmi2/ChFmuToolsImport.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

void CreateVehicleFMU(FmuChronoUnit& vehicle_fmu,
                      const std::string& instance_name,
                      const std::string& fmu_filename,
                      const std::string& fmu_unpack_dir,
                      double step_size,
                      double start_time,
                      double stop_time,
                      const std::vector<std::string>& logCategories,
                      const std::string& out_path,
                      bool visible,
                      double fps) {
    try {
        vehicle_fmu.Load(fmi2Type::fmi2CoSimulation, fmu_filename, fmu_unpack_dir);
    } catch (std::exception&) {
        throw;
    }
    std::cout << "Vehicle FMU version:  " << vehicle_fmu.GetVersion() << std::endl;
    std::cout << "Vehicle FMU platform: " << vehicle_fmu.GetTypesPlatform() << std::endl;

    // Instantiate FMU
    try {
        vehicle_fmu.Instantiate(instance_name, false, visible);
    } catch (std::exception&) {
        throw;
    }

    // Set debug logging
    vehicle_fmu.SetDebugLogging(fmi2True, logCategories);

    // Initialize FMU
    vehicle_fmu.SetupExperiment(fmi2False, 0.0,         // define tolerance
                                start_time,             // start time
                                fmi2False, stop_time);  // use stop time

    // Set I/O fixed parameters
    vehicle_fmu.SetVariable("out_path", out_path);
    vehicle_fmu.SetVariable("fps", fps, FmuVariable::Type::Real);

    // Set fixed parameters - use vehicle JSON files from the Chrono::Vehicle data directory
    std::string data_path = "../data/vehicle/";
    std::string vehicle_JSON = vehicle::GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json");

    vehicle_fmu.SetVariable("data_path", data_path);
    vehicle_fmu.SetVariable("vehicle_JSON", vehicle_JSON);
    vehicle_fmu.SetVariable("step_size", step_size, FmuVariable::Type::Real);
}

// -----------------------------------------------------------------------------

class DriverSystem {
  public:
    DriverSystem(ChSystem& sys, const std::string& path_filename);
    void Synchronize(double time, FmuChronoUnit& vehicle_fmu);
    void DoStep(double time, double step_size);

    void SetTargetSpeed(double speed) { target_speed = speed; }

    const ChVector3d& GetInitLoc() const { return init_loc; }
    double GetInitYaw() const { return init_yaw; }
    const ChFrameMoving<>& GetRefFrame() const { return ref_frame; }

    double GetSteering() const { return steering; }
    double GetThrottle() const { return throttle; }
    double GetBraking() const { return braking; }

  private:
    std::shared_ptr<chrono::vehicle::ChPathSteeringController> steeringPID;
    std::shared_ptr<chrono::vehicle::ChSpeedController> speedPID;
    double target_speed;
    double throttle_threshold;
    ChVector3d init_loc;
    double init_yaw;
    chrono::ChFrameMoving<> ref_frame;
    double steering;
    double throttle;
    double braking;
};

DriverSystem::DriverSystem(ChSystem& sys, const std::string& path_filename)
    : target_speed(10), steering(0), braking(0), throttle(0) {
    auto path = ChBezierCurve::Read(path_filename, false);

    speedPID = chrono_types::make_shared<ChSpeedController>();
    steeringPID = chrono_types::make_shared<ChPathSteeringController>(path);
    throttle_threshold = 0.2;

    double look_ahead_dist = 5.0;
    double Kp_steering = 0.8;
    double Ki_steering = 0.0;
    double Kd_steering = 0.0;
    double Kp_speed = 0.4;
    double Ki_speed = 0.0;
    double Kd_speed = 0.0;

    steeringPID->SetLookAheadDistance(look_ahead_dist);
    steeringPID->SetGains(Kp_steering, Ki_steering, Kd_steering);
    speedPID->SetGains(Kp_speed, Ki_speed, Kd_speed);

    auto point0 = path->GetPoint(0);
    auto point1 = path->GetPoint(1);
    init_loc = point0;
    init_yaw = std::atan2(point1.y() - point0.y(), point1.x() - point0.x());
}

void DriverSystem::Synchronize(double time, FmuChronoUnit& vehicle_fmu) {
    vehicle_fmu.GetFrameMovingVariable("ref_frame", ref_frame);

    vehicle_fmu.SetVariable("steering", steering, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("throttle", throttle, FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("braking", braking, FmuVariable::Type::Real);
}

void DriverSystem::DoStep(double time, double step_size) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = speedPID->Advance(ref_frame, target_speed, time, step_size);
    ChClampValue(out_speed, -1.0, 1.0);

    if (out_speed > 0) {
        // Vehicle moving too slow
        braking = 0;
        throttle = out_speed;
    } else if (throttle > throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        braking = 0;
        throttle = 1 + out_speed;
    } else {
        // Vehicle moving too fast: apply brakes
        braking = -out_speed;
        throttle = 0;
    }

    // Set the steering value based on the output from the steering controller.
    double out_steering = steeringPID->Advance(ref_frame, time, step_size);
    ChClampValue(out_steering, -1.0, 1.0);
    steering = out_steering;
}

// -----------------------------------------------------------------------------

class TireSystem {
  public:
    TireSystem(ChSystem& sys, const std::string& tire_JSON);
    void Synchronize(double time, FmuChronoUnit& vehicle_fmu, ChTerrain& terrain);
    void DoStep(double time, double step_size);

  private:
    class Wheel : public ChWheel {
      public:
        Wheel() : ChWheel("tire_wheel"), m_inertia(ChVector3d(0)) {}
        virtual double GetWheelMass() const override { return 0; }
        virtual const ChVector3d& GetWheelInertia() const override { return m_inertia; }
        virtual double GetRadius() const override { return 1; }
        virtual double GetWidth() const override { return 1; }

      private:
        ChVector3d m_inertia;
    };

    std::array<std::string, 4> ids;
    std::array<std::shared_ptr<ChTire>, 4> tires;
};

TireSystem::TireSystem(ChSystem& sys, const std::string& tire_JSON) {
    ids = {"wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"};

    for (int i = 0; i < 4; i++) {
        auto spindle = chrono_types::make_shared<ChBody>();
        sys.AddBody(spindle);

        auto wheel = chrono_types::make_shared<Wheel>();
        wheel->Initialize(nullptr, spindle, LEFT);

        tires[i] = ReadTireJSON(tire_JSON);
        tires[i]->Initialize(wheel);
        wheel->SetTire(tires[i]);
    }
}

void TireSystem::Synchronize(double time, FmuChronoUnit& vehicle_fmu, ChTerrain& terrain) {
    for (int i = 0; i < 4; i++) {
        // Get wheel state from vehicle FMU
        WheelState state;
        vehicle_fmu.GetVecVariable(ids[i] + ".pos", state.pos);
        vehicle_fmu.GetQuatVariable(ids[i] + ".rot", state.rot);
        vehicle_fmu.GetVecVariable(ids[i] + ".lin_vel", state.lin_vel);
        vehicle_fmu.GetVecVariable(ids[i] + ".ang_vel", state.ang_vel);

        // Get tire force
        auto force = tires[i]->ReportTireForce(&terrain);

        // Set spindle/wheel state and synchronize tire
        auto spindle = tires[i]->GetWheel()->GetSpindle();
        spindle->SetPos(state.pos);
        spindle->SetRot(state.rot);
        spindle->SetLinVel(state.lin_vel);
        spindle->SetAngVelParent(state.ang_vel);
        tires[i]->Synchronize(time, terrain);

        // Set tire force on vehicle FMU
        vehicle_fmu.SetVecVariable(ids[i] + ".point", force.point);
        vehicle_fmu.SetVecVariable(ids[i] + ".force", force.force);
        vehicle_fmu.SetVecVariable(ids[i] + ".moment", force.moment);
    }
}

void TireSystem::DoStep(double time, double step_size) {
    for (int i = 0; i < 4; i++) {
        tires[i]->Advance(step_size);
    }
}

// -----------------------------------------------------------------------------

class PowertrainSystem {
  public:
    PowertrainSystem(ChSystem& sys, const std::string& engine_json, const std::string& transmission_json);
    void Synchronize(double time, FmuChronoUnit& vehicle_fmu, double throttle);
    void DoStep(double time, double step_size);

  private:
    std::shared_ptr<chrono::vehicle::ChEngine> engine;
    std::shared_ptr<chrono::vehicle::ChTransmission> transmission;
    std::shared_ptr<chrono::vehicle::ChPowertrainAssembly> powertrain;

    class Chassis : public ChChassis {
      public:
        Chassis() : ChChassis("chassis") {}
        virtual std::string GetTemplateName() const override { return ""; }
        virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsysd(); }
        virtual void EnableCollision(bool state) override {}

        virtual double GetBodyMass() const override { return 1; }
        virtual ChFrame<> GetBodyCOMFrame() const override { return ChFramed(); }
        virtual ChMatrix33<> GetBodyInertia() const override { return ChMatrix33<>(1); }
    };
};

PowertrainSystem::PowertrainSystem(ChSystem& sys,
                                   const std::string& engine_JSON,
                                   const std::string& transmission_JSON) {
    engine = ReadEngineJSON(engine_JSON);
    transmission = ReadTransmissionJSON(transmission_JSON);
    powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);

    auto chassis = chrono_types::make_shared<Chassis>();
    chassis->Initialize(&sys, ChCoordsysd(), 0.0);
    chassis->SetFixed(true);

    powertrain->Initialize(chassis);
}

void PowertrainSystem::Synchronize(double time, FmuChronoUnit& vehicle_fmu, double throttle) {
    // Get driveshaft speed from vehicle FMU
    double driveshaft_speed;
    vehicle_fmu.GetVariable("driveshaft_speed", driveshaft_speed, FmuVariable::Type::Real);

    // Synchronize the powertrain assembly
    DriverInputs driver_inputs;
    driver_inputs.m_braking = 0;
    driver_inputs.m_clutch = 0;
    driver_inputs.m_steering = 0;
    driver_inputs.m_throttle = throttle;
    powertrain->Synchronize(time, driver_inputs, driveshaft_speed);

    // Set the driveshaft torque on vehicle FMU
    vehicle_fmu.SetVariable("driveshaft_torque", transmission->GetOutputDriveshaftTorque(), FmuVariable::Type::Real);

    // Set the engine and transmission reaction torques on vehicle FMU
    vehicle_fmu.SetVariable("engine_reaction", engine->GetChassisReactionTorque(), FmuVariable::Type::Real);
    vehicle_fmu.SetVariable("transmission_reaction", transmission->GetChassisReactionTorque(), FmuVariable::Type::Real);
}

void PowertrainSystem::DoStep(double time, double step_size) {
    powertrain->Advance(step_size);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << filesystem::path(argv[0]).filename() << std::endl;
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n" << std::endl;

#ifdef FMU_EXPORT_SUPPORT
    // Use the FMU generated in current build
    std::string vehicle_fmu_model_identifier = "FMU2_WheeledVehicle";
    std::string vehicle_fmu_dir = CHRONO_VEHICLE_FMU_DIR + vehicle_fmu_model_identifier + std::string("/");
    std::string vehicle_fmu_filename = vehicle_fmu_dir + vehicle_fmu_model_identifier + std::string(".fmu");
#else
    // Expect fully qualified FMU filename as program argument
    if (argc != 2) {
        std::cout << "Usage: ./demo_VEH_FMI2_WheeledVehicle_a [vehicle_FMU_filename]" << std::endl;
        return 1;
    }
    std::string vehicle_fmu_filename = argv[1];
#endif

    // FMU unpack directory
    std::string vehicle_unpack_dir = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_unpack_vehicle/");

    // Names of FMU instance
    std::string vehicle_instance_name = "WheeledVehicleFMU";

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "./DEMO_WHEELEDVEHICLE_FMI_COSIM_A";
    std::string vehicle_out_dir = out_dir + "/" + vehicle_instance_name;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(vehicle_out_dir))) {
        std::cout << "Error creating directory " << vehicle_out_dir << std::endl;
        return 1;
    }

    // Create external subsystems (terrain, driver, powertrain, and tires)
    ChSystemSMC sys;
    FlatTerrain terrain_sys(0.0, 0.8f);
    DriverSystem driver_sys(sys, vehicle::GetDataFile("paths/ISO_double_lane_change.txt"));
    TireSystem tire_sys(sys, vehicle::GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json"));
    ////PowertrainSystem powertrain_sys(sys, vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json"),
    ////                                vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json"));
    PowertrainSystem powertrain_sys(sys, vehicle::GetDataFile("hmmwv/powertrain/HMMWV_EngineSimpleMap.json"),
                                    vehicle::GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json"));

    driver_sys.SetTargetSpeed(12);

    // Create vehicle FMU
    std::vector<std::string> logCategories = {"logAll"};

    double start_time = 0;
    double stop_time = 16;
    double step_size = 1e-3;

    bool vehicle_visible = true;
    double fps = 60;
    bool save_img = false;

    ////std::cout << "Vehicle FMU filename: >" << vehicle_fmu_filename << "<" << std::endl;
    ////std::cout << "Vehicle FMU unpack directory: >" << vehicle_unpack_dir << "<" << std::endl;

    FmuChronoUnit vehicle_fmu;
    try {
        CreateVehicleFMU(vehicle_fmu,                                                      //
                         vehicle_instance_name, vehicle_fmu_filename, vehicle_unpack_dir,  //
                         step_size, start_time, stop_time,                                 //
                         logCategories, vehicle_out_dir, vehicle_visible, fps);            //
    } catch (std::exception& e) {
        std::cout << "ERROR loading vehicle FMU: " << e.what() << "\n";
        return 1;
    }

    // Initialize FMU
    vehicle_fmu.EnterInitializationMode();
    {
        vehicle_fmu.SetVecVariable("init_loc", driver_sys.GetInitLoc());
        vehicle_fmu.SetVariable("init_yaw", driver_sys.GetInitYaw(), FmuVariable::Type::Real);
    }
    vehicle_fmu.ExitInitializationMode();

    // Enable/disable saving snapshots
    vehicle_fmu.SetVariable("save_img", save_img);

    // Initialize output
    utils::ChWriterCSV csv;
    csv.SetDelimiter(" ");

    // Co-simulation loop
    double time = 0;
    ChTimer timer;
    timer.start();

    while (time < stop_time) {
        std::cout << "\r" << sys.GetChTime() << "\r";

        ////driver_sys.SetTargetSpeed(12);

        // Exchange data between vehicle FMU and external subsystems
        driver_sys.Synchronize(time, vehicle_fmu);
        tire_sys.Synchronize(time, vehicle_fmu, terrain_sys);
        powertrain_sys.Synchronize(time, vehicle_fmu, driver_sys.GetThrottle());

        // Save output
        csv << time << driver_sys.GetRefFrame().GetPos() << std::endl;

        // Advance FMU
        auto status_vehicle = vehicle_fmu.DoStep(time, step_size, fmi2True);
        if (status_vehicle == fmi2Discard)
            break;

        // Advance external subsystems
        driver_sys.DoStep(time, step_size);
        tire_sys.DoStep(time, step_size);
        powertrain_sys.DoStep(time, step_size);
        sys.DoStepDynamics(step_size);

        time += step_size;
    }

    timer.stop();
    std::cout << "Sim time: " << time << std::endl;
    std::cout << "Run time: " << timer() << std::endl;

    std::string out_file = out_dir + "/vehicle.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/vehicle.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("x");
    gplot.SetLabelY("y");
    gplot.SetTitle("Vehicle path");
    gplot.Plot(out_file, 2, 3, "path", " with lines lt 1 lw 2");
#endif

    return 0;
}
