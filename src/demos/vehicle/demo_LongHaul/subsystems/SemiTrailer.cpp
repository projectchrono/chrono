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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Trailer for the tractor-trailer vehicle model.
//
// =============================================================================

#include "subsystems/SemiTrailer.h"
#include "subsystems/SemiTrailer_axle.h"
#include "subsystems/SemiTrailer_wheel.h"
#include "subsystems/SemiTrailer_brake.h"

using namespace chrono;
using namespace chrono::vehicle;

SemiTrailer::SemiTrailer(ChSystem* mysystem, const bool fixed) {
    // Create the chassis and its connector
    m_chassis = chrono_types::make_shared<SemiTrailer_chassis>("Chassis");
    m_connector = chrono_types::make_shared<SemiTrailer_connector>("Connector");

    // Create the axle subsystems (suspension, wheels, and brakes)
    m_axles.resize(3);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("MidSusp");
    m_axles[2]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_ML");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_MR");

    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_RL");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_ML");
    m_axles[1]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_MR");
    m_axles[2]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_RL");
    m_axles[2]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_RR");
}

// Initialize the trailer relative to the chassis of the pulling vehicle
void SemiTrailer::Initialize(std::shared_ptr<ChChassis> frontChassis, const ChVector<>& location) {
    // Initialize the trailer chassis and its connector
    m_chassis->Initialize(frontChassis, location, WheeledCollisionFamily::CHASSIS);
    m_connector->Initialize(frontChassis, m_chassis);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-6.32, 0, 0), ChVector<>(0), 0.0);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-7.63, 0, 0), ChVector<>(0), 0.0);
    m_axles[2]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-8.94, 0, 0), ChVector<>(0), 0.0);
}

// Initialize the given tire and attach to the specified wheel
void SemiTrailer::InitializeTire(std::shared_ptr<ChTire> tire,
                                 std::shared_ptr<ChWheel> wheel,
                                 VisualizationType tire_vis,
                                 ChTire::CollisionType tire_coll) {
    wheel->SetTire(tire);
    tire->Initialize(wheel);
    tire->SetVisualizationType(tire_vis);
    tire->SetCollisionType(tire_coll);
}

// Set visualization type for the various subsystems
void SemiTrailer::SetChassisVisualizationType(chrono::vehicle::VisualizationType vis) {
    m_chassis->SetVisualizationType(vis);
}

void SemiTrailer::SetSuspensionVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        axle->m_suspension->SetVisualizationType(vis);
    }
}

void SemiTrailer::SetWheelVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            wheel->SetVisualizationType(vis);
        }
    }
}

// Get suspension spring forces
double SemiTrailer::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double SemiTrailer::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double SemiTrailer::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// Get shock damping forces
double SemiTrailer::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double SemiTrailer::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double SemiTrailer::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// Synchronize the trailer subsystem at the specified time
void SemiTrailer::Synchronize(double time, double braking, const ChTerrain& terrain) {
    // Synchronize the trailer's axle subsystems
    // (this applies tire forces to suspension spindles and braking input)
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Synchronize(time, terrain);
            axle->Synchronize(braking);
        }
    }
}

// Advance state of the trailer subsystem by the specified step
void SemiTrailer::Advance(double step) {
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Advance(step);
        }
    }
}
