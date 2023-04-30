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
// Authors: Radu Serban
// =============================================================================
//
// Brake for tracked vehicles modeled using a clutch between two shafts.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"

namespace chrono {
namespace vehicle {

ChTrackBrakeShafts::ChTrackBrakeShafts(const std::string& name) : ChTrackBrake(name), m_braking(0) {}

ChTrackBrakeShafts::~ChTrackBrakeShafts() {
    auto sys = m_shaft->GetSystem();
    if (sys) {
        sys->Remove(m_shaft);
        sys->Remove(m_clutch);
    }
}

void ChTrackBrakeShafts::Initialize(std::shared_ptr<ChChassis> chassis, std::shared_ptr<ChSprocket> sprocket) {
    ChTrackBrake::Initialize(chassis, sprocket);

    // Create and initialize the brake shaft
    m_shaft = chrono_types::make_shared<ChShaft>();
    m_shaft->SetNameString(m_name + "_shaft");
    m_shaft->SetInertia(GetShaftInertia());
    chassis->GetSystem()->AddShaft(m_shaft);

    // Create and initialize the connection between the brake shaft and the chassis
    auto connection = chrono_types::make_shared<ChShaftsBody>();
    connection->Initialize(m_shaft, chassis->GetBody(), ChVector<>(0, 1, 0));
    chassis->GetSystem()->Add(connection);

    // Create and initialize the brake clutch (set as unlocked)
    m_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_clutch->SetTorqueLimit(GetMaxBrakingTorque());
    m_clutch->Initialize(m_shaft, sprocket->GetAxle());
    m_clutch->SetModulation(0);
    chassis->GetSystem()->Add(m_clutch);
}

void ChTrackBrakeShafts::Synchronize(double braking) {
    m_braking = braking;
    m_clutch->SetModulation(braking);
}

}  // end namespace vehicle
}  // end namespace chrono
