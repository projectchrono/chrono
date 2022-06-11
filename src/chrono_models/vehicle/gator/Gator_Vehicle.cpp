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
// Authors: Radu Serban
// =============================================================================
//
// Gator full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/gator/Gator_Vehicle.h"
#include "chrono_models/vehicle/gator/Gator_Chassis.h"
#include "chrono_models/vehicle/gator/Gator_SingleWishbone.h"
#include "chrono_models/vehicle/gator/Gator_RigidSuspension.h"
#include "chrono_models/vehicle/gator/Gator_BrakeSimple.h"
#include "chrono_models/vehicle/gator/Gator_BrakeShafts.h"
#include "chrono_models/vehicle/gator/Gator_RackPinion.h"
#include "chrono_models/vehicle/gator/Gator_SimpleDriveline.h"
#include "chrono_models/vehicle/gator/Gator_Driveline2WD.h"
#include "chrono_models/vehicle/gator/Gator_Wheel.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_Vehicle::Gator_Vehicle(const bool fixed,
                             DrivelineTypeWV driveline_type,
                             BrakeType brake_type,
                             ChContactMethod contact_method,
                             CollisionType chassis_collision_type)
    : ChWheeledVehicle("Gator", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, driveline_type, brake_type, chassis_collision_type);
}

Gator_Vehicle::Gator_Vehicle(ChSystem* system,
                             const bool fixed,
                             DrivelineTypeWV driveline_type,
                             BrakeType brake_type,
                             CollisionType chassis_collision_type)
    : ChWheeledVehicle("Gator", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, driveline_type, brake_type, chassis_collision_type);
}

void Gator_Vehicle::Create(bool fixed,
                           DrivelineTypeWV driveline_type,
                           BrakeType brake_type,
                           CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Gator_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Gator_SingleWishbone>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Gator_RigidSuspension>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Gator_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Gator_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Gator_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Gator_Wheel>("Wheel_RR");

    // Note: brakes only on rear axle
    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[1]->m_brake_left = chrono_types::make_shared<Gator_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Gator_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[1]->m_brake_left = chrono_types::make_shared<Gator_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Gator_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Gator_RackPinion>("Steering");

    // Create the driveline
    switch (driveline_type) {
        default:
        case DrivelineTypeWV::SIMPLE:
            m_driveline = chrono_types::make_shared<Gator_SimpleDriveline>("Driveline");
            break;
        case DrivelineTypeWV::RWD:
            m_driveline = chrono_types::make_shared<Gator_Driveline2WD>("Driveline");
            break;
    }
}

Gator_Vehicle::~Gator_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Gator_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = ChVector<>(0.92, 0, 0.08);
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0.97, 0, 0), ChVector<>(0), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-0.97, 0, 0), ChVector<>(0), 0.0, m_omega[2],
                           m_omega[3]);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp_indexes = {1};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Gator_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSingleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double Gator_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSingleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double Gator_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSingleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// -----------------------------------------------------------------------------
void Gator_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSingleWishbone>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSingleWishbone>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
// -----------------------------------------------------------------------------
void Gator_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        GetLog() << "Velocity [m/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
