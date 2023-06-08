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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/powertrain/ChManualTransmissionShafts.h"

namespace chrono {
namespace vehicle {

ChManualTransmissionShafts::ChManualTransmissionShafts(const std::string& name) : ChManualTransmission(name) {}

ChManualTransmissionShafts::~ChManualTransmissionShafts() {
    auto sys = m_motorshaft->GetSystem();
    if (sys) {
        sys->Remove(m_motorshaft);
        sys->Remove(m_driveshaft);
        sys->Remove(m_transmissionblock);
        sys->Remove(m_transmissionblock_to_body);
        sys->Remove(m_gears);
        sys->Remove(m_clutchShaft);
        sys->Remove(m_clutch);
    }
}

// -----------------------------------------------------------------------------
void ChManualTransmissionShafts::Initialize(std::shared_ptr<ChChassis> chassis) {
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

    // Create the clutch, replacing the torque converter above. The clutch connects the crankshaft with what,
    // for lack of a better word, I call clutchshaft now.
    m_clutchShaft = chrono_types::make_shared<ChShaft>();  // TODO what's the name of this shaft?
    m_clutchShaft->SetInertia(GetIngearShaftInertia());    // TODO for now re-use this one, naming?
    sys->AddShaft(m_clutchShaft);
    m_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_clutch->Initialize(m_motorshaft, m_clutchShaft);
    m_clutch->SetTorqueLimit(GetClutchTorqueLimit());
    m_clutch->SetModulation(1);
    sys->Add(m_clutch);

    // Create a gearbox, i.e a transmission ratio constraint between two shafts.
    // Note that differently from the basic ChShaftsGear, this also provides
    // the possibility of transmitting a reaction torque to the box (the truss).
    m_gears = chrono_types::make_shared<ChShaftsGearbox>();
    m_gears->Initialize(m_clutchShaft, m_driveshaft, chassis->GetBody(), dir_transmissionblock);
    m_gears->SetTransmissionRatio(m_current_gear_ratio);
    sys->Add(m_gears);
}

// -----------------------------------------------------------------------------
void ChManualTransmissionShafts::OnGearShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

void ChManualTransmissionShafts::OnNeutralShift() {
    if (m_gears)
        m_gears->SetTransmissionRatio(m_current_gear_ratio);
}

// -----------------------------------------------------------------------------
void ChManualTransmissionShafts::Synchronize(double time,
                                             const DriverInputs& driver_inputs,
                                             double motorshaft_torque,
                                             double driveshaft_speed) {
    // Enforce inputs from engine (torque) and driveline (speed)
    m_motorshaft->SetAppliedTorque(motorshaft_torque);
    m_driveshaft->SetPos_dt(driveshaft_speed);

    // Clutch
    m_clutch->SetModulation(1.0 - driver_inputs.m_clutch);
}

double ChManualTransmissionShafts::GetOutputDriveshaftTorque() const {
    return m_gears->GetTorqueReactionOn2();
}

double ChManualTransmissionShafts::GetOutputMotorshaftSpeed() const {
    return m_motorshaft->GetPos_dt();
}

}  // end namespace vehicle
}  // end namespace chrono
