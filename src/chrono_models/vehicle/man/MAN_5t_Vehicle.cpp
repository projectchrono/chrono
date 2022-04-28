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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He, Rainer Gericke
// =============================================================================
//
// MAN 5t full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

#include "chrono_models/vehicle/man/MAN_5t_Vehicle.h"
#include "chrono_models/vehicle/man/MAN_5t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeShafts.h"
#include "chrono_models/vehicle/man/MAN_5t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_5t_BellcrankSolid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_5t_Driveline4WD.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleDriveline.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_Vehicle::MAN_5t_Vehicle(const bool fixed,
                               BrakeType brake_type,
                               ChContactMethod contact_method,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("MAN_5t", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

MAN_5t_Vehicle::MAN_5t_Vehicle(ChSystem* system,
                               const bool fixed,
                               BrakeType brake_type,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("MAN_5t", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

void MAN_5t_Vehicle::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<MAN_5t_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems (suspension + wheels + brakes)
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<MAN_5t_BellcrankSolid3LinkAxle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<MAN_5t_Solid3LinkAxle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<MAN_5t_RotaryArm>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<MAN_5t_Driveline4WD>("Driveline");
}

MAN_5t_Vehicle::~MAN_5t_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = ChVector<>(0, 0, 0.0);  // 0.4 0 0.4
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0, 0, 0), ChVector<>(0), 0.0, m_omega[0],
                           m_omega[1]);
    const double twin_tire_dist = 0.0;  // Michelin for 305/85 R22.5
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-4.5, 0, 0), ChVector<>(0), twin_tire_dist,
                           m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp_indexes;
    driven_susp_indexes.resize(2);
    driven_susp_indexes[0] = 0;
    driven_susp_indexes[1] = 1;
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double MAN_5t_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double MAN_5t_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double MAN_5t_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double MAN_5t_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double MAN_5t_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double MAN_5t_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void MAN_5t_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidAxle>(m_axles[0]->m_suspension)->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidAxle>(m_axles[1]->m_suspension)->LogHardpointLocations(ChVector<>(0, 0, 0), false);

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
void MAN_5t_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        GetLog() << "Deformation [m]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

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

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
