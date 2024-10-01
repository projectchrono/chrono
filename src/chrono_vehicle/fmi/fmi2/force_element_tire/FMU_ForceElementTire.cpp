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
// Co-simulation FMU encapsulating a "force element" (handling) tire system.
//
// =============================================================================

#include <cassert>
#include <algorithm>

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "FMU_ForceElementTire.h"

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
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    step_size = 1e-3;

    out_path = ".";

    wheel_load.point = ChVector3d(0.0);
    wheel_load.force = ChVector3d(0.0);
    wheel_load.moment = ChVector3d(0.0);

    query_point = ChVector3d(0.0);

    // Get default JSON file from FMU resources
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    tire_JSON = resources_dir + "/TMeasyTire.json";

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&tire_JSON, "tire_JSON", FmuVariable::Type::String, "1", "tire JSON",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set FIXED PARAMETERS for this FMU (I/O)
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU (wheel state)
    AddFmuVecVariable(wheel_state.pos, "wheel_state.pos", "m", "wheel position",                      //
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuQuatVariable(wheel_state.rot, "wheel_state.rot", "1", "wheel rotation",                     //
                       FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVecVariable(wheel_state.lin_vel, "wheel_state.lin_vel", "m/s", "wheel linear velocity",     //
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVecVariable(wheel_state.ang_vel, "wheel_state.ang_vel", "rad/s", "wheel angular velocity",  //
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //

    // Set CONTINUOUS OUTPUTS for this FMU (wheel load)
    AddFmuVecVariable(wheel_load.point, "wheel_load.point", "m", "wheel load application point",     //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                      FmuVariable::InitialType::exact);                                              //
    AddFmuVecVariable(wheel_load.force, "wheel_load.force", "N", "wheel load applied force",         //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                      FmuVariable::InitialType::exact);                                              //
    AddFmuVecVariable(wheel_load.moment, "wheel_load.moment", "Nm", "wheel load applied moment",     //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                      FmuVariable::InitialType::exact);                                              //

    // Set CONTINUOUS OUTPUTS for this FMU (terrain query point)
    AddFmuVecVariable(query_point, "query_point", "m", "terrain query point",                        //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                      FmuVariable::InitialType::exact);                                              //

    // Set CONTINUOUS INPUTS for this FMU (terrain information)
    AddFmuVariable(&terrain_height, "terrain_height", FmuVariable::Type::Real, "m", "terrain height",        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);             //
    AddFmuVecVariable(terrain_normal, "terrain_normal", "1", "terrain normal",                               //
                      FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);          //
    AddFmuVariable(&terrain_mu, "terrain_mu", FmuVariable::Type::Real, "1", "terrain friction coefficient",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);             //

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeTire(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateTireOutputs(); });
}

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

void FmuComponent::CreateTire() {
    std::cout << "Create tire FMU" << std::endl;
    std::cout << " Tire JSON: " << tire_JSON << std::endl;

    tire = ReadTireJSON(tire_JSON);
    assert(std::dynamic_pointer_cast<ChForceElementTire>(tire));

    //// TODO: does any other method make sense here?
    //// This tire FMU uses a locally-flat terrain patch that is updated at each synchronization time.
    tire->SetCollisionType(ChTire::CollisionType::SINGLE_POINT);

    auto spindle = chrono_types::make_shared<ChBody>();
    sys.AddBody(spindle);

    wheel = chrono_types::make_shared<Wheel>();
    wheel->Initialize(nullptr, spindle, LEFT);

    wheel->SetTire(tire);
    tire->Initialize(wheel);
}

void FmuComponent::SynchronizeTire(double time) {
    // Set the state of the spindle body
    auto spindle = wheel->GetSpindle();
    spindle->SetPos(wheel_state.pos);
    spindle->SetRot(wheel_state.rot);
    spindle->SetLinVel(wheel_state.lin_vel);
    spindle->SetAngVelParent(wheel_state.ang_vel);

    // Update the local terrain
    terrain.height = terrain_height;
    terrain.normal = terrain_normal;
    terrain.mu = terrain_mu;

    // Synchronize the tire (this will query the dummy terrain)
    tire->Synchronize(time, terrain);
}

void FmuComponent::CalculateTireOutputs() {
    // Set current load on associated wheel
    wheel_load = tire->ReportTireForce(&terrain);

    // Set current query point for terrain information
    query_point = wheel_state.pos;
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    CreateTire();
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                              std::min(communicationStepSize, step_size));

        tire->Advance(h);

        ////sendToLog("time: " + std::to_string(m_time) + "\n", fmi2Status::fmi2OK, "logAll");

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}
