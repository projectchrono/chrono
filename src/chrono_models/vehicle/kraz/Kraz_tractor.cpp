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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Semitractor for the long haul vehicle model based on Kraz 64431 data
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Brake.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Driveline.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Wheel.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Steering.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_FrontSuspension.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_RearSuspension.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Chassis.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
Kraz_tractor::Kraz_tractor(bool fixed, ChContactMethod contactMethod) : ChWheeledVehicle("KrazTractor", contactMethod) {
    Create(fixed);
}

Kraz_tractor::Kraz_tractor(ChSystem* system, bool fixed) : ChWheeledVehicle("KrazTractor", system) {
    Create(fixed);
}

void Kraz_tractor::Create(bool fixed) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Kraz_tractor_Chassis>("Chassis");

    // Create the axle subsystems
    m_axles.resize(3);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Kraz_tractor_FrontSuspension>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Kraz_tractor_RearSuspension>("RearSusp1");
    m_axles[2]->m_suspension = chrono_types::make_shared<Kraz_tractor_RearSuspension>("RearSusp2");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(4);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RL1i");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RR1i");
    m_axles[1]->m_wheels[2] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RL1o");
    m_axles[1]->m_wheels[3] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RR1o");

    m_axles[2]->m_wheels.resize(4);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RL2i");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RR2i");
    m_axles[2]->m_wheels[2] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RL2o");
    m_axles[2]->m_wheels[3] = chrono_types::make_shared<Kraz_tractor_Wheel>("Wheel_RR2o");

    m_axles[0]->m_brake_left = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_RL1");
    m_axles[1]->m_brake_right = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_RR1");
    m_axles[2]->m_brake_left = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_RL2");
    m_axles[2]->m_brake_right = chrono_types::make_shared<Kraz_tractor_Brake>("Brake_RR2");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Kraz_tractor_Steering>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<Kraz_tractor_Driveline>("driveline");
}

// -----------------------------------------------------------------------------
void Kraz_tractor::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    m_steerings[0]->Initialize(m_chassis, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0.0, 0, 0), ChVector<>(0), 0.0);
    const double twin_tire_dist = 0.33528;  // Michelin for 12.00 R 20
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-4.08, 0, 0), ChVector<>(0), twin_tire_dist);
    m_axles[2]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-5.48, 0, 0), ChVector<>(0), twin_tire_dist);

    // Initialize the driveline subsystem (6x4 = rear axles are driven)
    std::vector<int> driven_susp = {1, 2};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
double Kraz_tractor::GetSpringForce(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        default:
            return -1;
    }
}

double Kraz_tractor::GetSpringLength(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        default:
            return -1;
    }
}

double Kraz_tractor::GetSpringDeformation(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)
                ->GetSpringDeformation(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
double Kraz_tractor::GetShockForce(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
        default:
            return -1;
    }
}

double Kraz_tractor::GetShockLength(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
        default:
            return -1;
    }
}

double Kraz_tractor::GetShockVelocity(int axle, VehicleSide side) const {
    switch (axle) {
        case 0:
            return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)
                ->GetShockVelocity(side);
        case 1:
        case 2:
            return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Kraz_tractor::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR#1 suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR#2 suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[2]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);

    GetLog() << "\n\n";
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf (are they?)
// -----------------------------------------------------------------------------
void Kraz_tractor::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear1-left, rear1-right, rear2-left, rear2-right)\n";
        GetLog() << "Length [inch]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << GetSpringLength(2, LEFT) << "  "
                 << GetSpringLength(2, RIGHT) << "\n";
        GetLog() << "Deformation [inch]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "  "
                 << GetSpringDeformation(2, LEFT) << "  " << GetSpringDeformation(2, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << GetSpringForce(2, LEFT) << "  "
                 << GetSpringForce(2, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear1-left, rear1-right, rear2-left, rear2-right)\n";
        GetLog() << "Length [inch]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << GetShockLength(2, LEFT) << "  "
                 << GetShockLength(2, RIGHT) << "\n";
        GetLog() << "Velocity [inch/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << GetShockVelocity(2, LEFT) << "  "
                 << GetShockVelocity(2, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << GetShockForce(2, LEFT) << "  "
                 << GetShockForce(2, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
