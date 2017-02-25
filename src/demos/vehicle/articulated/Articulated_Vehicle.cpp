// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Alessandro Tasora
// =============================================================================
//
// Articulated vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "articulated/Articulated_Chassis.h"
#include "articulated/Articulated_Vehicle.h"

#include "chrono_models/vehicle/generic/Generic_SolidAxle.h"
#include "chrono_models/vehicle/generic/Generic_MultiLink.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_RackPinion.h"
#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Articulated_Vehicle::Articulated_Vehicle(const bool fixed,
                                         SuspensionType suspType,
                                         ChMaterialSurfaceBase::ContactMethod contactMethod)
    : ChWheeledVehicle(contactMethod), m_suspType(suspType) {
    // -------------------------------------------
    // Create the chassis subsystem
    // -------------------------------------------
    m_chassis = std::make_shared<Articulated_Chassis>("Chassis");

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(2);

    assert(m_suspType == SuspensionType::SOLID_AXLE || m_suspType == SuspensionType::MULTI_LINK);

    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            m_suspensions[0] = std::make_shared<Generic_SolidAxle>("FrontSusp");
            m_suspensions[1] = std::make_shared<Generic_SolidAxle>("RearSusp");
            break;
        case SuspensionType::MULTI_LINK:
            m_suspensions[0] = std::make_shared<Generic_MultiLink>("FrontSusp");
            m_suspensions[1] = std::make_shared<Generic_MultiLink>("RearSusp");
            break;
        default:
            break;
    }

    // -----------------------------
    // Create the steering subsystem
    // -----------------------------
    m_steerings.resize(1);
    m_steerings[0] = std::make_shared<Generic_RackPinion>("Steering");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(4);
    m_wheels[0] = std::make_shared<Generic_Wheel>("Wheel_FL");
    m_wheels[1] = std::make_shared<Generic_Wheel>("Wheel_FR");
    m_wheels[2] = std::make_shared<Generic_Wheel>("Wheel_RL");
    m_wheels[3] = std::make_shared<Generic_Wheel>("Wheel_RR");

    // --------------------
    // Create the driveline
    // --------------------
    m_driveline = std::make_shared<Generic_Driveline2WD>("driveline");

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(4);
    m_brakes[0] = std::make_shared<Generic_BrakeSimple>("Brake_FL");
    m_brakes[1] = std::make_shared<Generic_BrakeSimple>("Brake_FR");
    m_brakes[2] = std::make_shared<Generic_BrakeSimple>("Brake_RL");
    m_brakes[3] = std::make_shared<Generic_BrakeSimple>("Brake_RR");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel);

    // Initialize the steering subsystem (specify the steering subsystem's frame
    // relative to the chassis reference frame).
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

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.6914, 0, 0), m_steerings[0]->GetSteeringLink());
    m_suspensions[1]->Initialize(m_chassis->GetBody(), ChVector<>(-1.6865, 0, 0), m_chassis->GetBody());

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
    m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
    m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp(1, 1);
    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp);

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
    m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
    m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Articulated_Vehicle::GetSpringForce(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
        default:
            return -1;
    }
}

double Articulated_Vehicle::GetSpringLength(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
        default:
            return -1;
    }
}

double Articulated_Vehicle::GetSpringDeformation(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Articulated_Vehicle::GetShockForce(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
        default:
            return -1;
    }
}

double Articulated_Vehicle::GetShockLength(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
        default:
            return -1;
    }
}

double Articulated_Vehicle::GetShockVelocity(const WheelID& wheel_id) const {
    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
        case SuspensionType::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Articulated_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    switch (m_suspType) {
        case SuspensionType::SOLID_AXLE:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        case SuspensionType::MULTI_LINK:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_suspensions[0])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_suspensions[1])->LogHardpointLocations(ChVector<>(0, 0, 0), true);
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
void Articulated_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetSpringLength(FRONT_LEFT) << "  " << GetSpringLength(FRONT_RIGHT)
                 << "  " << GetSpringLength(REAR_LEFT) << "  " << GetSpringLength(REAR_RIGHT) << "\n";
        GetLog() << "Deformation [inch]  " << GetSpringDeformation(FRONT_LEFT) << "  "
                 << GetSpringDeformation(FRONT_RIGHT) << "  " << GetSpringDeformation(REAR_LEFT) << "  "
                 << GetSpringDeformation(REAR_RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetSpringForce(FRONT_LEFT) << "  " << GetSpringForce(FRONT_RIGHT) << "  "
                 << GetSpringForce(REAR_LEFT) << "  " << GetSpringForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetShockLength(FRONT_LEFT) << "  " << GetShockLength(FRONT_RIGHT) << "  "
                 << GetShockLength(REAR_LEFT) << "  " << GetShockLength(REAR_RIGHT) << "\n";
        GetLog() << "Velocity [inch/s]   " << GetShockVelocity(FRONT_LEFT) << "  " << GetShockVelocity(FRONT_RIGHT)
                 << "  " << GetShockVelocity(REAR_LEFT) << "  " << GetShockVelocity(REAR_RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetShockForce(FRONT_LEFT) << "  " << GetShockForce(FRONT_RIGHT) << "  "
                 << GetShockForce(REAR_LEFT) << "  " << GetShockForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}
