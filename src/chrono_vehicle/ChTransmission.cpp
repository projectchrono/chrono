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
// Base class for a vehicle transmission.
//
// =============================================================================

#include "chrono_vehicle/ChTransmission.h"

namespace chrono {
namespace vehicle {

#define CHRONO_NEUTRAL_GEAR_RATIO 1e-20

ChTransmission::ChTransmission(const std::string& name)
    : ChPart(name), m_current_gear(-1), m_current_gear_ratio(CHRONO_NEUTRAL_GEAR_RATIO) {}

void ChTransmission::Initialize(std::shared_ptr<ChChassis> chassis) {
    // Let the derived class specify the gear ratios
    std::vector<double> fwd;
    double rev;
    SetGearRatios(fwd, rev);
    assert(fwd.size() > 0);
    if (rev > 0)
        rev *= -1;
    m_gear_ratios.push_back(rev);
    m_gear_ratios.insert(m_gear_ratios.end(), fwd.begin(), fwd.end());

    // Initialize the transmission in 1st gear
    SetGear(1);

    // Mark as initialized
    m_initialized = true;
}

void ChTransmission::InitializeInertiaProperties() {
    m_mass = 0;
    m_inertia = ChMatrix33<>(0);
    m_com = ChFrame<>();
    m_xform = ChFrame<>();
}

void ChTransmission::UpdateInertiaProperties() {}

void ChTransmission::SetGear(int gear) {
    assert(gear >= -1);
    assert(gear < m_gear_ratios.size());

    if (m_current_gear == gear)
        return;

    m_current_gear = gear;
    if (m_current_gear < 0) {
        m_current_gear_ratio = m_gear_ratios[0];
        OnGearShift();
    }
    else if (m_current_gear > 0 && m_current_gear < m_gear_ratios.size()) {
        m_current_gear_ratio = m_gear_ratios[m_current_gear];
        OnGearShift();
    }
    else {
        m_current_gear_ratio = CHRONO_NEUTRAL_GEAR_RATIO;
        OnNeutralShift();
    }
}

void ChAutomaticTransmission::SetDriveMode(DriveMode mode) {
    if (m_drive_mode == mode)
        return;

    m_drive_mode = mode;

    switch (m_drive_mode) {
        case DriveMode::FORWARD:
            SetGear(1);
            break;
        case DriveMode::NEUTRAL:
            SetGear(0);
            break;
        case DriveMode::REVERSE:
            SetGear(-1);
            break;
    }
}

// -----------------------------------------------------------------------------

ChAutomaticTransmission::ChAutomaticTransmission(const std::string& name)
    : ChTransmission(name), m_drive_mode(DriveMode::FORWARD), m_shift_mode(ShiftMode::AUTOMATIC) {}

void ChAutomaticTransmission::ShiftUp() {
    if (m_shift_mode == ShiftMode::MANUAL &&   //
        m_drive_mode == DriveMode::FORWARD &&  //
        m_current_gear < GetMaxGear()) {
        SetGear(m_current_gear + 1);
    }
}

void ChAutomaticTransmission::ShiftDown() {
    if (m_shift_mode == ShiftMode::MANUAL &&   //
        m_drive_mode == DriveMode::FORWARD &&  //
        m_current_gear > 1) {
        SetGear(m_current_gear - 1);
    }
}

// -----------------------------------------------------------------------------

ChManualTransmission::ChManualTransmission(const std::string& name) : ChTransmission(name) {}

void ChManualTransmission::ShiftUp() {
    if (m_current_gear < GetMaxGear()) {
        SetGear(m_current_gear + 1);
    }
}

void ChManualTransmission::ShiftDown() {
    if (m_current_gear >= 0) {
        SetGear(m_current_gear - 1);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
