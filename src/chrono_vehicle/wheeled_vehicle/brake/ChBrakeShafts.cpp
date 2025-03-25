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
// Brake for wheeled vehicles modeled using a clutch between two shafts.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

namespace chrono {
namespace vehicle {

ChBrakeShafts::ChBrakeShafts(const std::string& name) : ChBrake(name), m_modulation(0), m_locked(false) {}

ChBrakeShafts::~ChBrakeShafts() {
    if (!m_initialized)
        return;

    auto sys = m_shaft->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_shaft);
    sys->Remove(m_clutch);
}

void ChBrakeShafts::Construct(std::shared_ptr<ChChassis> chassis,
                              std::shared_ptr<ChSuspension> suspension,
                              VehicleSide side) {
    // Create and initialize the brake shaft
    m_shaft = chrono_types::make_shared<ChShaft>();
    m_shaft->SetName(m_name + "_shaft");
    m_shaft->SetTag(m_obj_tag);
    m_shaft->SetInertia(GetShaftInertia());
    chassis->GetSystem()->AddShaft(m_shaft);

    // Create and initialize the connection between the brake shaft and a "fixed" body
    auto body = suspension->GetBrakeBody(side);
    if (!body)
        body = chassis->GetBody();
    auto connection = chrono_types::make_shared<ChShaftBodyRotation>();
    connection->Initialize(m_shaft, body, ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(connection);

    // Create and initialize the brake clutch (set as unlocked)
    m_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_clutch->SetTorqueLimit(GetMaxBrakingTorque());
    m_clutch->Initialize(m_shaft, suspension->GetAxle(side));
    m_clutch->SetModulation(0);
    chassis->GetSystem()->Add(m_clutch);
}

void ChBrakeShafts::Synchronize(double time, double braking) {
    m_modulation = braking;
    m_clutch->SetModulation(braking);
    //// TODO: more here?
}

}  // end namespace vehicle
}  // end namespace chrono
