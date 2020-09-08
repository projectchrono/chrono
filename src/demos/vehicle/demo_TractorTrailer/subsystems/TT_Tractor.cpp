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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Alessandro Tasora
// =============================================================================
//
// Tractor for the tractor-trailer vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"

#include "chrono_models/vehicle/generic/Generic_SolidAxle.h"
#include "chrono_models/vehicle/generic/Generic_MultiLink.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_RackPinion.h"
#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"

#include "subsystems/TT_Chassis.h"
#include "subsystems/TT_Tractor.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TT_Tractor::TT_Tractor(const bool fixed, SuspensionType suspType, ChContactMethod contactMethod)
    : ChWheeledVehicle("Tractor", contactMethod), m_suspType(suspType) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<TT_Chassis>("Chassis");

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    assert(m_suspType == SuspensionType::SOLID_AXLE || m_suspType == SuspensionType::MULTI_LINK);

    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_SolidAxle>("FrontSusp");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_SolidAxle>("RearSusp");
            break;
        case SuspensionType::MULTI_LINK:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_MultiLink>("FrontSusp");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_MultiLink>("RearSusp");
            break;
        default:
            break;
    }

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RR");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Generic_RackPinion>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<Generic_Driveline2WD>("driveline");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TT_Tractor::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset;
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            offset = ChVector<>(1.60, 0, -0.07);
            break;
        case SuspensionType::MULTI_LINK:
            offset = ChVector<>(1.65, 0, -0.12);
            break;
        default:
            break;
    }
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.6914, 0, 0), ChVector<>(0),
                           m_steerings[0]->GetSteeringLink(), 0, 0.0);
    m_axles[1]->Initialize(m_chassis->GetBody(), ChVector<>(-1.6865, 0, 0), ChVector<>(0), m_chassis->GetBody(), -1,
                           0.0);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp = {1};
    m_driveline->Initialize(m_chassis->GetBody(), m_axles, driven_susp);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double TT_Tractor::GetSpringForce(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        default:
            return -1;
    }
}

double TT_Tractor::GetSpringLength(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        default:
            return -1;
    }
}

double TT_Tractor::GetSpringDeformation(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double TT_Tractor::GetShockForce(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockForce(side);
        default:
            return -1;
    }
}

double TT_Tractor::GetShockLength(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockLength(side);
        default:
            return -1;
    }
}

double TT_Tractor::GetShockVelocity(int axle, VehicleSide side) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void TT_Tractor::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        case SuspensionType::MULTI_LINK:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        default:
            break;
    }

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void TT_Tractor::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        GetLog() << "Deformation [inch]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        GetLog() << "Velocity [inch/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}
