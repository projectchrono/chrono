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
// Engine model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/powertrain/ChEngineShafts.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the motor block, i.e. the
// direction of the crankshaft, in chassis local coords. This is needed because
// ChShaftsBody could transfer rolling torque to the chassis.
// -----------------------------------------------------------------------------
ChEngineShafts::ChEngineShafts(const std::string& name, const ChVector<>& dir_motor_block)
    : ChEngine(name), m_dir_motor_block(dir_motor_block) {}

ChEngineShafts::~ChEngineShafts() {
    auto sys = m_engine->GetSystem();
    if (sys) {
        sys->Remove(m_motorblock);
        sys->Remove(m_motorblock_to_body);
        sys->Remove(m_engine);
        sys->Remove(m_engine_losses);
        sys->Remove(m_motorshaft);
    }
}

double ChEngineShafts::GetOutputMotorshaftTorque() const {
    return m_engine->GetTorqueReactionOn1() + m_engine_losses->GetTorqueReactionOn1();
}

// -----------------------------------------------------------------------------
void ChEngineShafts::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChEngine::Initialize(chassis);

    assert(chassis->GetBody()->GetSystem());
    auto sys = chassis->GetSystem();

    // Create the motorshaft which represents the crankshaft plus flywheel.
    m_motorshaft = chrono_types::make_shared<ChShaft>();
    m_motorshaft->SetInertia(GetMotorshaftInertia());
    sys->AddShaft(m_motorshaft);

    // Create the motor block.
    // ChShaftsThermalEngine connects this motor block to the motorshaft and applies the engine torque between them.
    m_motorblock = chrono_types::make_shared<ChShaft>();
    m_motorblock->SetInertia(GetMotorBlockInertia());
    sys->AddShaft(m_motorblock);

    // Create  a connection between the motor block and the 3D rigid body that represents the chassis.
    // This allows to get the effect of the car 'rolling' when the longitudinal engine accelerates suddenly.
    m_motorblock_to_body = chrono_types::make_shared<ChShaftsBody>();
    m_motorblock_to_body->Initialize(m_motorblock, chassis->GetBody(), m_dir_motor_block);
    sys->Add(m_motorblock_to_body);

    // Create a thermal engine model between motor block and motorshaft (both receive the torque, but with opposite
    // sign).
    m_engine = chrono_types::make_shared<ChShaftsThermalEngine>();
    m_engine->Initialize(m_motorshaft, m_motorblock);
    sys->Add(m_engine);

    // The thermal engine requires a torque curve
    auto mTw = chrono_types::make_shared<ChFunction_Recorder>();
    SetEngineTorqueMap(mTw);
    m_engine->SetTorqueCurve(mTw);

    // Create an engine brake model to model engine losses due to inner friction, turbulence, etc.
    // Without this, the engine at 0% throttle in neutral position would rotate forever at constant speed.
    m_engine_losses = chrono_types::make_shared<ChShaftsThermalEngine>();
    m_engine_losses->Initialize(m_motorshaft, m_motorblock);
    sys->Add(m_engine_losses);

    // The engine brake model requires a torque curve
    auto mTw_losses = chrono_types::make_shared<ChFunction_Recorder>();
    SetEngineLossesMap(mTw_losses);
    m_engine_losses->SetTorqueCurve(mTw_losses);
}

// -----------------------------------------------------------------------------
void ChEngineShafts::Synchronize(double time, const DriverInputs& driver_inputs, double motorshaft_speed) {
    // Apply shaft speed
    m_motorshaft->SetPos_dt(motorshaft_speed);

    // Update the throttle level in the thermal engine
    m_engine->SetThrottle(driver_inputs.m_throttle);
}

}  // end namespace vehicle
}  // end namespace chrono
