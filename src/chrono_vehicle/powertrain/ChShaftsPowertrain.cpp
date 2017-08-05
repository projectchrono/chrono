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
    : ChPowertrain(name), m_dir_motor_block(dir_motor_block), m_last_time_gearshift(0), m_gear_shift_latency(0.5) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    assert(chassis);
    assert(driveshaft);
    assert(chassis->GetSystem());

    ChSystem* my_system = chassis->GetSystem();

    // Let the derived class specify the gear ratios
    SetGearRatios(m_gear_ratios);
    assert(m_gear_ratios.size() > 1);
    m_current_gear = 1;

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // In this case it is the motor block. This because the ChShaftsThermalEngine
    // needs two 1dof items to apply the torque in-between them (the other will be
    // the crankshaft). In simpler models, one could just use SetShaftFixed() on
    // this object, but here we prefer to leave it free and use a ChShaftsBody
    // constraint to connect it to the car chassis (so the car can 'roll' when
    // pressing the throttle, like in muscle cars)
    m_motorblock = std::make_shared<ChShaft>();
    m_motorblock->SetInertia(GetMotorBlockInertia());
    my_system->Add(m_motorblock);

    // CREATE  a connection between the motor block and the 3D rigid body that
    // represents the chassis. This allows to get the effect of the car 'rolling'
    // when the longitudinal engine accelerates suddenly.
    m_motorblock_to_body = std::make_shared<ChShaftsBody>();
    m_motorblock_to_body->Initialize(m_motorblock, chassis, m_dir_motor_block);
    my_system->Add(m_motorblock_to_body);

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the crankshaft plus flywheel.
    m_crankshaft = std::make_shared<ChShaft>();
    m_crankshaft->SetInertia(GetCrankshaftInertia());
    my_system->Add(m_crankshaft);

    // CREATE  a thermal engine model between motor block and crankshaft (both
    // receive the torque, but with opposite sign).
    m_engine = std::make_shared<ChShaftsThermalEngine>();
    m_engine->Initialize(m_crankshaft, m_motorblock);
    my_system->Add(m_engine);
    // The thermal engine requires a torque curve:
    auto mTw = std::make_shared<ChFunction_Recorder>();
    SetEngineTorqueMap(mTw);
    m_engine->SetTorqueCurve(mTw);

    // CREATE  an engine brake model that represents the losses of the engine because
    // of inner friction/turbulence/etc. Without this, the engine at 0% throttle
    // in neutral position would rotate forever at constant speed.
    m_engine_losses = std::make_shared<ChShaftsThermalEngine>();
    m_engine_losses->Initialize(m_crankshaft, m_motorblock);
    my_system->Add(m_engine_losses);
    // The engine brake model requires a torque curve:
    auto mTw_losses = std::make_shared<ChFunction_Recorder>();
    SetEngineLossesMap(mTw_losses);
    m_engine_losses->SetTorqueCurve(mTw_losses);

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that collects all inertias from torque converter to the gear.
    m_shaft_ingear = std::make_shared<ChShaft>();
    m_shaft_ingear->SetInertia(GetIngearShaftInertia());
    my_system->Add(m_shaft_ingear);

    // CREATE a torque converter and connect the shafts:
    // A (input),B (output), C(truss stator).
    // The input is the m_crankshaft, output is m_shaft_ingear; for stator, reuse the motor block 1D item.
    m_torqueconverter = std::make_shared<ChShaftsTorqueConverter>();
    m_torqueconverter->Initialize(m_crankshaft, m_shaft_ingear, m_motorblock);
    my_system->Add(m_torqueconverter);
    // To complete the setup of the torque converter, a capacity factor curve is needed:
    auto mK = std::make_shared<ChFunction_Recorder>();
    SetTorqueConverterCapacityFactorMap(mK);
    m_torqueconverter->SetCurveCapacityFactor(mK);
    // To complete the setup of the torque converter, a torque ratio curve is needed:
    auto mT = std::make_shared<ChFunction_Recorder>();
    SetTorqeConverterTorqueRatioMap(mT);
    m_torqueconverter->SetCurveTorqueRatio(mT);

    // CREATE a gearbox, i.e a transmission ratio constraint between two
    // shafts. Note that differently from the basic ChShaftsGear, this also provides
    // the possibility of transmitting a reaction torque to the box (the truss).
    m_gears = std::make_shared<ChShaftsGearbox>();
    m_gears->Initialize(m_shaft_ingear, driveshaft, chassis, m_dir_motor_block);
    m_gears->SetTransmissionRatio(m_gear_ratios[m_current_gear]);
    my_system->Add(m_gears);

    // -------
    // Finally, update the gear ratio according to the selected gear in the
    // array of gear ratios:
    SetSelectedGear(1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::SetSelectedGear(int igear) {
    assert(igear >= 0);
    assert(igear < m_gear_ratios.size());

    m_current_gear = igear;
    if (m_gears)
        m_gears->SetTransmissionRatio(m_gear_ratios[igear]);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::SetDriveMode(ChPowertrain::DriveMode mmode) {
    if (m_drive_mode == mmode)
        return;

    m_drive_mode = mmode;

    if (!m_gears)
        return;

    switch (m_drive_mode) {
        case FORWARD:
            SetSelectedGear(1);
            break;
        case NEUTRAL:
            m_gears->SetTransmissionRatio(1e20);
            break;
        case REVERSE:
            SetSelectedGear(0);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // Just update the throttle level in the thermal engine
    m_engine->SetThrottle(throttle);

    // To avoid bursts of gear shifts, do nothing if the last shift was too recent
    if (time - m_last_time_gearshift < m_gear_shift_latency)
        return;

    // Shift the gear if needed, automatically shifting up or down with
    // a very simple logic, for instance as in the following fixed latency
    // state machine:
    if (m_drive_mode != FORWARD)
        return;

    double gearshaft_speed = m_shaft_ingear->GetPos_dt();

    if (gearshaft_speed > 2500 * CH_C_2PI / 60.0) {
        // upshift if possible
        if (m_current_gear + 1 < m_gear_ratios.size()) {
            GetLog() << "SHIFT UP " << m_current_gear << " -> " << m_current_gear + 1 << "\n";
            SetSelectedGear(m_current_gear + 1);
            m_last_time_gearshift = time;
        }
    } else if (gearshaft_speed < 1500 * CH_C_2PI / 60.0) {
        // downshift if possible
        if (m_current_gear - 1 > 0) {
            GetLog() << "SHIFT DOWN " << m_current_gear << " -> " << m_current_gear - 1 << "\n";
            SetSelectedGear(m_current_gear - 1);
            m_last_time_gearshift = time;
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
