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
// Base class for a wheeled trailer system.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"

namespace chrono {
namespace vehicle {

ChWheeledTrailer::ChWheeledTrailer(const std::string& name, ChSystem* system): m_name(name) {}

void ChWheeledTrailer::Initialize(std::shared_ptr<ChChassis> frontChassis) {
    m_chassis->Initialize(frontChassis, WheeledCollisionFamily::CHASSIS);
    m_connector->Initialize(frontChassis, m_chassis);
}

void ChWheeledTrailer::InitializeTire(std::shared_ptr<ChTire> tire,
                                 std::shared_ptr<ChWheel> wheel,
                                 VisualizationType tire_vis,
                                 ChTire::CollisionType tire_coll) {
    wheel->SetTire(tire);
    tire->Initialize(wheel);
    tire->SetVisualizationType(tire_vis);
    tire->SetCollisionType(tire_coll);
}

void ChWheeledTrailer::SetChassisVisualizationType(VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void ChWheeledTrailer::SetSuspensionVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        axle->m_suspension->SetVisualizationType(vis);
    }
}

void ChWheeledTrailer::SetWheelVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            wheel->SetVisualizationType(vis);
        }
    }
}

void ChWheeledTrailer::SetTireVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            if (wheel->GetTire())
                wheel->GetTire()->SetVisualizationType(vis);
        }
    }
}

// Synchronize the trailer subsystem at the specified time
void ChWheeledTrailer::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    // Synchronize the trailer's axle subsystems
    // (this applies tire forces to suspension spindles and braking input)
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Synchronize(time, terrain);
            axle->Synchronize(time, driver_inputs);
        }
    }
}

// Advance state of the trailer subsystem by the specified step
void ChWheeledTrailer::Advance(double step) {
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Advance(step);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
