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
// Trailer for the Kraz tractor-trailer vehicle model.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_trailer.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Chassis.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Suspension.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Wheel.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Brake.h"

namespace chrono {
namespace vehicle {
namespace kraz {

Kraz_trailer::Kraz_trailer(ChSystem* system, CollisionType chassis_collision_type)
    : ChWheeledTrailer("KrazTrailer", system) {
    // Create the chassis and its connector
    m_chassis = chrono_types::make_shared<Kraz_trailer_Chassis>("Chassis", chassis_collision_type);
    m_connector = chrono_types::make_shared<Kraz_trailer_Connector>("Connector");

    // Create the axle subsystems (suspension, wheels, and brakes)
    m_axles.resize(3);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Kraz_trailer_Suspension>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Kraz_trailer_Suspension>("MidSusp");
    m_axles[2]->m_suspension = chrono_types::make_shared<Kraz_trailer_Suspension>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_ML");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_MR");

    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_RL");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<Kraz_trailer_Wheel>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_ML");
    m_axles[1]->m_brake_right = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_MR");
    m_axles[2]->m_brake_left = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_RL");
    m_axles[2]->m_brake_right = chrono_types::make_shared<Kraz_trailer_Brake>("Brake_RR");
}

// Initialize the trailer relative to the chassis of the pulling vehicle
void Kraz_trailer::Initialize(std::shared_ptr<ChChassis> frontChassis) {
    // Initialize the trailer chassis and its connector
    ChWheeledTrailer::Initialize(frontChassis);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-6.32, 0, 0), ChVector3d(0), 0.0);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-7.63, 0, 0), ChVector3d(0), 0.0);
    m_axles[2]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-8.94, 0, 0), ChVector3d(0), 0.0);
}

// Get suspension spring forces
double Kraz_trailer::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double Kraz_trailer::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double Kraz_trailer::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// Get shock damping forces
double Kraz_trailer::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double Kraz_trailer::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double Kraz_trailer::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
