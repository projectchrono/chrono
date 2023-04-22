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
// Automatic transmission model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the motor block, i.e. the
// direction of the crankshaft, in chassis local coords. This is needed because
// ChShaftsBody could transfer rolling torque to the chassis.
// -----------------------------------------------------------------------------
ChAutomaticTransmissionShafts::ChAutomaticTransmissionShafts(const std::string& name)
    : ChTransmission(name), m_last_time_gearshift(0), m_gear_shift_latency(0.5) {}

ChAutomaticTransmissionShafts::~ChAutomaticTransmissionShafts() {
    auto sys = m_torqueconverter->GetSystem();
    if (sys) {
        sys->Remove(m_motorshaft);
        sys->Remove(m_driveshaft);
        sys->Remove(m_transmissionblock);
        sys->Remove(m_transmissionblock_to_body);
        sys->Remove(m_torqueconverter);
        sys->Remove(m_shaft_ingear);
        sys->Remove(m_gears);
    }
}

// -----------------------------------------------------------------------------
void ChAutomaticTransmissionShafts::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChTransmission::Initialize(chassis);

    assert(chassis->GetBody()->GetSystem());
    ChSystem* sys = chassis->GetBody()->GetSystem();

    // Create the motorshaft and driveshaft
    m_motorshaft = chrono_types::make_shared<ChShaft>();
    m_motorshaft->SetInertia(GetMotorshaftInertia());
    sys->AddShaft(m_motorshaft);

    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->AddShaft(m_driveshaft);

    //// TODO: allow longitudinal/transversal transmission block?
    ////       get from engine?
    ChVector<> dir_transmissionblock(1, 0, 0);

    // Create the motor block.
    // ChShaftsThermalEngine connects this motor block to the motorshaft and applies the engine torque between them.
    m_transmissionblock = chrono_types::make_shared<ChShaft>();
    m_transmissionblock->SetInertia(GetTransmissionBlockInertia());
    sys->AddShaft(m_transmissionblock);

    // Create  a connection between the transmission block and the 3D rigid body that represents the chassis.
    m_transmissionblock_to_body = chrono_types::make_shared<ChShaftsBody>();
    m_transmissionblock_to_body->Initialize(m_transmissionblock, chassis->GetBody(), dir_transmissionblock);
    sys->Add(m_transmissionblock_to_body);

    // Cache the upshift and downshift speeds (in rad/s)
    m_upshift_speed = GetUpshiftRPM() * CH_C_2PI / 60.0;
    m_downshift_speed = GetDownshiftRPM() * CH_C_2PI / 60.0;

    // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that collects all inertias from torque converter to the gear.
    m_shaft_ingear = chrono_types::make_shared<ChShaft>();
    m_shaft_ingear->SetInertia(GetIngearShaftInertia());
    sys->AddShaft(m_shaft_ingear);

    // CREATE a torque converter and connect the shafts:
    // Input: motorshaft, output: shaft_ingear, stator: transmission block.
    m_torqueconverter = chrono_types::make_shared<ChShaftsTorqueConverter>();
    m_torqueconverter->Initialize(m_motorshaft, m_shaft_ingear, m_transmissionblock);
    sys->Add(m_torqueconverter);

    // To complete the setup of the torque converter, a capacity factor curve is needed:
    auto mK = chrono_types::make_shared<ChFunction_Recorder>();
    SetTorqueConverterCapacityFactorMap(mK);
    m_torqueconverter->SetCurveCapacityFactor(mK);

    // To complete the setup of the torque converter, a torque ratio curve is needed:
    auto mT = chrono_types::make_shared<ChFunction_Recorder>();
    SetTorqeConverterTorqueRatioMap(mT);
    m_torqueconverter->SetCurveTorqueRatio(mT);

    // Create a gearbox, i.e a transmission ratio constraint between two shafts.
    // Note that differently from the basic ChShaftsGear, this also provides
    // the possibility of transmitting a reaction torque to the box (the truss).
    m_gears = chrono_types::make_shared<ChShaftsGearbox>();
    m_gears->Initialize(m_shaft_ingear, m_driveshaft, chassis->GetBody(), dir_transmissionblock);
    m_gears->SetTransmissionRatio(m_current_gear_ratio);
    sys->Add(m_gears);
}

// -----------------------------------------------------------------------------
void ChAutomaticTransmissionShafts::OnGearShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

void ChAutomaticTransmissionShafts::OnNeutralShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

// -----------------------------------------------------------------------------
void ChAutomaticTransmissionShafts::Synchronize(double time,
                                                const DriverInputs& driver_inputs,
                                                double motorshaft_torque,
                                                double driveshaft_speed) {
    // Enforce inputs from engine (torque) and driveline (speed)
    m_motorshaft->SetAppliedTorque(motorshaft_torque);
    m_driveshaft->SetPos_dt(driveshaft_speed);

    // To avoid bursts of gear shifts, do nothing if the last shift was too recent
    if (time - m_last_time_gearshift < m_gear_shift_latency)
        return;

    // Automatic gear selection (fixed latency state machine)
    if (m_mode == Mode::AUTOMATIC && m_drive_mode == DriveMode::FORWARD) {
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

double ChAutomaticTransmissionShafts::GetOutputDriveshaftTorque() const {
    return m_gears->GetTorqueReactionOn2();
}

double ChAutomaticTransmissionShafts::GetOutputMotorshaftSpeed() const {
    return m_motorshaft->GetPos_dt();
}

}  // end namespace vehicle
}  // end namespace chrono
