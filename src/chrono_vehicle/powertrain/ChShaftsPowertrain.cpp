// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the motor block, i.e. the
// direction of the crankshaft, in chassis local coords. This is needed because
// ChShaftsBody could transfer rolling torque to the chassis.
// -----------------------------------------------------------------------------
ChShaftsPowertrain::ChShaftsPowertrain(const std::string& name, const ChVector<>& dir_motor_block)
    : ChPowertrain(name), m_dir_motor_block(dir_motor_block), m_last_time_gearshift(0), m_gear_shift_latency(0.5) {}

// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Initialize(std::shared_ptr<ChChassis> chassis, std::shared_ptr<ChDriveline> driveline) {
    ChPowertrain::Initialize(chassis, driveline);

    assert(chassis->GetBody()->GetSystem());
    ChSystem* my_system = chassis->GetBody()->GetSystem();

    // Cache the upshift and downshift speeds (in rad/s)
    m_upshift_speed = GetUpshiftRPM() * CH_C_2PI / 60.0;
    m_downshift_speed = GetDownshiftRPM() * CH_C_2PI / 60.0;

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // In this case it is the motor block. This because the ChShaftsThermalEngine
    // needs two 1dof items to apply the torque in-between them (the other will be
    // the crankshaft). In simpler models, one could just use SetShaftFixed() on
    // this object, but here we prefer to leave it free and use a ChShaftsBody
    // constraint to connect it to the car chassis (so the car can 'roll' when
    // pressing the throttle, like in muscle cars)
    m_motorblock = chrono_types::make_shared<ChShaft>();
    m_motorblock->SetInertia(GetMotorBlockInertia());
    my_system->Add(m_motorblock);

    // CREATE  a connection between the motor block and the 3D rigid body that
    // represents the chassis. This allows to get the effect of the car 'rolling'
    // when the longitudinal engine accelerates suddenly.
    m_motorblock_to_body = chrono_types::make_shared<ChShaftsBody>();
    m_motorblock_to_body->Initialize(m_motorblock, chassis->GetBody(), m_dir_motor_block);
    my_system->Add(m_motorblock_to_body);

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the crankshaft plus flywheel.
    m_crankshaft = chrono_types::make_shared<ChShaft>();
    m_crankshaft->SetInertia(GetCrankshaftInertia());
    my_system->Add(m_crankshaft);

    // CREATE  a thermal engine model between motor block and crankshaft (both
    // receive the torque, but with opposite sign).
    m_engine = chrono_types::make_shared<ChShaftsThermalEngine>();
    m_engine->Initialize(m_crankshaft, m_motorblock);
    my_system->Add(m_engine);
    // The thermal engine requires a torque curve:
    auto mTw = chrono_types::make_shared<ChFunction_Recorder>();
    SetEngineTorqueMap(mTw);
    m_engine->SetTorqueCurve(mTw);

    // CREATE  an engine brake model that represents the losses of the engine because
    // of inner friction/turbulence/etc. Without this, the engine at 0% throttle
    // in neutral position would rotate forever at constant speed.
    m_engine_losses = chrono_types::make_shared<ChShaftsThermalEngine>();
    m_engine_losses->Initialize(m_crankshaft, m_motorblock);
    my_system->Add(m_engine_losses);
    // The engine brake model requires a torque curve:
    auto mTw_losses = chrono_types::make_shared<ChFunction_Recorder>();
    SetEngineLossesMap(mTw_losses);
    m_engine_losses->SetTorqueCurve(mTw_losses);

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that collects all inertias from torque converter to the gear.
    m_shaft_ingear = chrono_types::make_shared<ChShaft>();
    m_shaft_ingear->SetInertia(GetIngearShaftInertia());
    my_system->Add(m_shaft_ingear);

    // CREATE a torque converter and connect the shafts:
    // A (input),B (output), C(truss stator).
    // The input is the m_crankshaft, output is m_shaft_ingear; for stator, reuse the motor block 1D item.
    m_torqueconverter = chrono_types::make_shared<ChShaftsTorqueConverter>();
    m_torqueconverter->Initialize(m_crankshaft, m_shaft_ingear, m_motorblock);
    my_system->Add(m_torqueconverter);
    // To complete the setup of the torque converter, a capacity factor curve is needed:
    auto mK = chrono_types::make_shared<ChFunction_Recorder>();
    SetTorqueConverterCapacityFactorMap(mK);
    m_torqueconverter->SetCurveCapacityFactor(mK);
    // To complete the setup of the torque converter, a torque ratio curve is needed:
    auto mT = chrono_types::make_shared<ChFunction_Recorder>();
    SetTorqeConverterTorqueRatioMap(mT);
    m_torqueconverter->SetCurveTorqueRatio(mT);

    // CREATE a gearbox, i.e a transmission ratio constraint between two
    // shafts. Note that differently from the basic ChShaftsGear, this also provides
    // the possibility of transmitting a reaction torque to the box (the truss).
    m_gears = chrono_types::make_shared<ChShaftsGearbox>();
    m_gears->Initialize(m_shaft_ingear, driveline->GetDriveshaft(), chassis->GetBody(), m_dir_motor_block);
    m_gears->SetTransmissionRatio(m_current_gear_ratio);
    my_system->Add(m_gears);
}

// -----------------------------------------------------------------------------
void ChShaftsPowertrain::OnGearShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

void ChShaftsPowertrain::OnNeutralShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Synchronize(double time, double throttle) {
    // Just update the throttle level in the thermal engine
    m_engine->SetThrottle(throttle);

    // To avoid bursts of gear shifts, do nothing if the last shift was too recent
    if (time - m_last_time_gearshift < m_gear_shift_latency)
        return;

    // Automatic gear selection (fixed latency state machine)
    if (m_transmission_mode == TransmissionMode::AUTOMATIC && m_drive_mode == DriveMode::FORWARD) {
        double gearshaft_speed = m_shaft_ingear->GetPos_dt();
        if (gearshaft_speed > m_upshift_speed) {
            // upshift if possible
            if (m_current_gear < m_gear_ratios.size() - 1) {
                SetGear(m_current_gear + 1);
                m_last_time_gearshift = time;
            }
        } else if (gearshaft_speed < m_downshift_speed) {
            // downshift if possible
            if (m_current_gear > 1) {
                SetGear(m_current_gear - 1);
                m_last_time_gearshift = time;
            }
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
