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

ChTransmission::ChTransmission(const std::string& name)
    : ChPart(name),
      m_mode(Mode::AUTOMATIC),
      m_drive_mode(DriveMode::FORWARD),
      m_current_gear(-1),
      m_current_gear_ratio(1e20) {}

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

void ChTransmission::ShiftUp() {
    if (m_mode == Mode::MANUAL &&              //
        m_drive_mode == DriveMode::FORWARD &&  //
        m_current_gear < m_gear_ratios.size() - 1) {
        SetGear(m_current_gear + 1);
    }
}

void ChTransmission::ShiftDown() {
    if (m_mode == Mode::MANUAL &&              //
        m_drive_mode == DriveMode::FORWARD &&  //
        m_current_gear > 1) {
        SetGear(m_current_gear - 1);
    }
}

void ChTransmission::SetGear(int gear) {
    assert(gear >= 0);
    assert(gear < m_gear_ratios.size());

    m_current_gear = gear;
    m_current_gear_ratio = m_gear_ratios[m_current_gear];
    OnGearShift();
}

void ChTransmission::SetDriveMode(DriveMode mode) {
    if (m_drive_mode == mode)
        return;

    m_drive_mode = mode;

    switch (m_drive_mode) {
        case DriveMode::FORWARD:
            SetGear(1);
            break;
        case DriveMode::NEUTRAL:
            m_current_gear_ratio = 1e20;
            OnNeutralShift();
            break;
        case DriveMode::REVERSE:
            SetGear(0);
            break;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
