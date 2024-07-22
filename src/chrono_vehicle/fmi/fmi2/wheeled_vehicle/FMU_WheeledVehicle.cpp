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
// Co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels.
// The vehicle does not include an engine, transmission, or tires.
//
// =============================================================================

#include <cassert>
#include <map>
#include <algorithm>
#include <iomanip>

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "FMU_WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
fmu_tools::fmi2::FmuComponentBase* fmu_tools::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
                                                                        fmi2Type fmuType,
                                                                        fmi2String fmuGUID,
                                                                        fmi2String fmuResourceLocation,
                                                                        const fmi2CallbackFunctions* functions,
                                                                        fmi2Boolean visible,
                                                                        fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn),
      render_frame(0) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    g_acc = {0, 0, -9.8};
    driver_inputs = {0, 0, 0, 0};
    init_loc = {0, 0, 0};
    init_yaw = 0;

    engineblock_dir = {1, 0, 0};
    transmissionblock_dir = {1, 0, 0};

    driveshaft_speed = 0;
    driveshaft_torque = 0;
    engine_reaction = 0;
    transmission_reaction = 0;

    system_SMC = 1;
    step_size = 1e-3;

    out_path = ".";
    save_img = false;
    fps = 60;

    // Get default JSON files from the FMU resources directory
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    data_path = resources_dir + "/";
    vehicle_JSON = resources_dir + "/Vehicle.json";

    // Set wheel identifier strings
    wheel_data[0].identifier = "FL";
    wheel_data[1].identifier = "FR";
    wheel_data[2].identifier = "RL";
    wheel_data[3].identifier = "RR";

#ifdef CHRONO_IRRLICHT
    if (visible)
        vis_sys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
#else
    if (visible)
        std::cout << "The FMU was not built with run-time visualization support. Visualization disabled." << std::endl;
#endif

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&data_path, "data_path", FmuVariable::Type::String, "1", "vehicle data path",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVariable(&vehicle_JSON, "vehicle_JSON", FmuVariable::Type::String, "1", "vehicle JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);    //

    AddFmuVariable(&system_SMC, "system_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVecVariable(init_loc, "init_loc", "m", "initial location",                                //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial location Z",     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVecVariable(engineblock_dir, "engineblock_dir", "1", "engine block mounting direction",                    //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVecVariable(transmissionblock_dir, "transmissionblock_dir", "1", "transmission block mounting direction",  //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //

    AddFmuVecVariable(g_acc, "g_acc", "m/s2", "gravitational acceleration",                         //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set FIXED PARAMETERS for this FMU (I/O)
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&fps, "fps", FmuVariable::Type::Real, "1", "rendering frequency",             //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU (driver inputs)
    AddFmuVariable(&driver_inputs.m_steering, "steering", FmuVariable::Type::Real, "1", "steering input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_throttle, "throttle", FmuVariable::Type::Real, "1", "throttle input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_braking, "braking", FmuVariable::Type::Real, "1", "braking input",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_clutch, "clutch", FmuVariable::Type::Real, "1", "clutch input",        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    // Set DISCRETE INPUTS for this FMU (I/O)
    AddFmuVariable((int*)(&save_img), "save_img", FmuVariable::Type::Boolean, "1", "trigger saving images",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::discrete);               //

    // Set CONTINUOUS OUTPUTS for this FMU (vehicle reference frame)
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                          //
                              FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU (driveshaft speed and torque for co-simulation)
    AddFmuVariable(&driveshaft_torque, "driveshaft_torque", FmuVariable::Type::Real,              //
                   "Nm", "driveshaft motor torque",                                               //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&driveshaft_speed, "driveshaft_speed", FmuVariable::Type::Real,                //
                   "rad/s", "driveshaft angular speed",                                           //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //

    AddFmuVariable(&engine_reaction, "engine_reaction", FmuVariable::Type::Real,                  //
                   "Nm", "engine reaction torque",                                                //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&transmission_reaction, "transmission_reaction", FmuVariable::Type::Real,      //
                   "Nm", "transmission reaction torque",                                          //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU (wheel state and forces for co-simulation)
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state.lin_vel = VNULL;
        wheel_data[iw].state.ang_vel = VNULL;

        std::string prefix = "wheel_" + wheel_data[iw].identifier;

        AddFmuVecVariable(wheel_data[iw].load.point, prefix + ".point", "m", prefix + " load application point",  //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
        AddFmuVecVariable(wheel_data[iw].load.force, prefix + ".force", "N", prefix + " load applied force",      //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
        AddFmuVecVariable(wheel_data[iw].load.moment, prefix + ".moment", "Nm", prefix + " load applied moment",  //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

        AddFmuVecVariable(wheel_data[iw].state.pos, prefix + ".pos", "m", prefix + " position",                      //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
        AddFmuQuatVariable(wheel_data[iw].state.rot, prefix + ".rot", "1", prefix + " rotation",                     //
                           FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);            //
        AddFmuVecVariable(wheel_data[iw].state.lin_vel, prefix + ".lin_vel", "m/s", prefix + " linear velocity",     //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //
        AddFmuVecVariable(wheel_data[iw].state.ang_vel, prefix + ".ang_vel", "rad/s", prefix + " angular velocity",  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,              //
                          FmuVariable::InitialType::exact);                                                          //
    }

    // Specify variable dependencies
    DeclareVariableDependencies("ref_frame", {"init_loc", "init_yaw"});
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;
        DeclareVariableDependencies(prefix + ".pos", {"init_loc", "init_yaw"});
        DeclareVariableDependencies(prefix + ".rot", {"init_loc", "init_yaw"});
    }

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateVehicleOutputs(); });
}

void FmuComponent::CreateVehicle() {
    std::cout << "Create vehicle FMU" << std::endl;
    std::cout << " Data path:         " << data_path << std::endl;
    std::cout << " Vehicle JSON:      " << vehicle_JSON << std::endl;
    std::cout << " Initial location:  " << init_loc << std::endl;
    std::cout << " Initial yaw:       " << init_yaw << std::endl;

    vehicle::SetDataPath(data_path);

    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle_JSON,
                                                        system_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    vehicle->Initialize(ChCoordsys<>(init_loc + ChVector3d(0, 0, 0.5), QuatFromAngleZ(init_yaw)));

    // Initialize the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    // Cache vehicle wheels
    wheel_data[0].wheel = vehicle->GetWheel(0, VehicleSide::LEFT);
    wheel_data[1].wheel = vehicle->GetWheel(0, VehicleSide::RIGHT);
    wheel_data[2].wheel = vehicle->GetWheel(1, VehicleSide::LEFT);
    wheel_data[3].wheel = vehicle->GetWheel(1, VehicleSide::RIGHT);

    //// TODO: allow setting through FMU variables
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);
}

void FmuComponent::ConfigureSystem() {
    // Containing system
    auto system = vehicle->GetSystem();

    system->SetGravitationalAcceleration(g_acc);

    // Associate a collision system
    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Modify solver settings if the vehicle model contains bushings
    if (vehicle->HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);
        solver->SetVerbose(false);

        step_size = std::min(step_size, 2e-4);
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
}

void FmuComponent::SynchronizeVehicle(double time) {
    // 1. synchronize the vehicle system.
    // ATTENTION: this version of 'Synchronize' does not apply tire forces to the vehicle wheels.
    vehicle->Synchronize(time, driver_inputs);

    // 2. apply tire forces (received from outside) to wheel bodies.
    //  ATTENTION: this must be done AFTER vehicle synchronization (which may empty wheel force accumulators).
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].wheel->Synchronize(wheel_data[iw].load);
    }

    // 3. apply driveshaft torque (received from outside) to the driveline.
    vehicle->GetDriveline()->Synchronize(time, driver_inputs, driveshaft_torque);

    // 4. apply reaction torques from engine and transmission.
    //// TODO

    // 5. Synchronize the run-time visualization (if available and enabled)
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        vis_sys->Synchronize(time, driver_inputs);
    }
#endif
}

void FmuComponent::CalculateVehicleOutputs() {
    // Extract wheel states
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state = wheel_data[iw].wheel->GetState();
    }

    // Update the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    driveshaft_speed = vehicle->GetDriveline()->GetOutputDriveshaftSpeed();

    //// TODO - other vehicle outputs...
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    // Create the vehicle system
    CreateVehicle();

    // Configure Chrono system
    ConfigureSystem();

    // Initialize runtime visualization (if requested and if available)
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        std::cout << " Enable run-time visualization" << std::endl;

        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetJPEGQuality(100);
        vis_sys->SetWindowTitle("Wheeled Vehicle FMU (FMI 2.0)");
        vis_sys->SetWindowSize(800, 800);
        vis_sys->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
        vis_sys->AddGrid(0.5, 0.5, 2000, 400, ChCoordsys<>(init_loc, QuatFromAngleZ(init_yaw)),
                         ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
    }
#endif
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                              std::min(communicationStepSize, step_size));
        vehicle->Advance(h);

#ifdef CHRONO_IRRLICHT
        if (vis_sys) {
            auto status = vis_sys->Run();
            if (!status)
                return fmi2Discard;
            vis_sys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vis_sys->Render();
            vis_sys->RenderFrame(ref_frame);
            vis_sys->EndScene();

            if (save_img && m_time >= render_frame / fps) {
                std::ostringstream filename;
                filename << out_path << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".bmp";
                vis_sys->WriteImageToFile(filename.str());
                render_frame++;
            }

            vis_sys->Advance(h);
        }
#endif

        ////sendToLog("time: " + std::to_string(m_time) + "\n", fmi2Status::fmi2OK, "logAll");

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}
