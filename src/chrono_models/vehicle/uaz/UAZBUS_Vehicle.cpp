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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// UAZBUS full vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Vehicle::UAZBUS_Vehicle(const bool fixed,
                               SteeringType steering_model,
                               ChMaterialSurface::ContactMethod contact_method,
                               ChassisCollisionType chassis_collision_type)
    : ChWheeledVehicle("UAZBUS", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, steering_model, chassis_collision_type);
}

UAZBUS_Vehicle::UAZBUS_Vehicle(ChSystem* system,
                               const bool fixed,
                               SteeringType steering_model,
                               ChassisCollisionType chassis_collision_type)
    : ChWheeledVehicle("UAZBUS", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, steering_model, chassis_collision_type);
}

void UAZBUS_Vehicle::Create(bool fixed, SteeringType steering_model, ChassisCollisionType chassis_collision_type) {
    // -------------------------------------------
    // Create the chassis subsystem
    // -------------------------------------------
    m_chassis = std::make_shared<UAZBUS_Chassis>("Chassis", fixed, chassis_collision_type);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(2);
    m_suspensions[0] = std::make_shared<UAZBUS_ToeBarLeafspringAxle>("FrontSusp");
    m_suspensions[1] = std::make_shared<UAZBUS_LeafspringAxle>("RearSusp");

    // -----------------------------
    // Create the steering subsystem
    // -----------------------------
    m_steerings.resize(1);
    m_steerings[0] = std::make_shared<UAZBUS_RotaryArm>("Steering");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(4);
    m_wheels[0] = std::make_shared<UAZBUS_WheelLeft>("Wheel_FL");
    m_wheels[1] = std::make_shared<UAZBUS_WheelRight>("Wheel_FR");
    m_wheels[2] = std::make_shared<UAZBUS_WheelLeft>("Wheel_RL");
    m_wheels[3] = std::make_shared<UAZBUS_WheelRight>("Wheel_RR");

    // --------------------
    // Create the driveline
    // --------------------
    m_driveline = std::make_shared<UAZBUS_Driveline4WD>("Driveline");
    // m_driveline = std::make_shared<UAZBUS_SimpleDriveline>("Driveline");

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(4);
    m_brakes[0] = std::make_shared<UAZBUS_BrakeSimpleFront>("Brake_FL");
    m_brakes[1] = std::make_shared<UAZBUS_BrakeSimpleFront>("Brake_FR");
    m_brakes[2] = std::make_shared<UAZBUS_BrakeSimpleRear>("Brake_RL");
    m_brakes[3] = std::make_shared<UAZBUS_BrakeSimpleRear>("Brake_RR");
}

UAZBUS_Vehicle::~UAZBUS_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the steering subsystem (specify the steering subsystem's frame
    // relative to the chassis reference frame).
    ChVector<> offset = ChVector<>(0, 0, 0);
    ChQuaternion<> rotation = Q_from_AngAxis(0, ChVector<>(0, 1, 0));
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, rotation);

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(0, 0, 0), m_steerings[0]->GetSteeringLink(), 0,
                                 m_omega[0], m_omega[1]);
    m_suspensions[1]->Initialize(m_chassis->GetBody(), ChVector<>(-2.3, 0, 0), m_chassis->GetBody(), -1, m_omega[2],
                                 m_omega[3]);

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
    m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
    m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    driven_susp_indexes[0] = 1;
    driven_susp_indexes[1] = 1;

    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp_indexes);

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
    m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
    m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double UAZBUS_Vehicle::GetSpringForce(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetSpringForce(wheel_id.side());
}

double UAZBUS_Vehicle::GetSpringLength(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetSpringLength(wheel_id.side());
}

double UAZBUS_Vehicle::GetSpringDeformation(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetSpringDeformation(wheel_id.side());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double UAZBUS_Vehicle::GetShockForce(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetShockForce(wheel_id.side());
}

double UAZBUS_Vehicle::GetShockLength(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetShockLength(wheel_id.side());
}

double UAZBUS_Vehicle::GetShockVelocity(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[wheel_id.axle()])
        ->GetShockVelocity(wheel_id.side());
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void UAZBUS_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[0])
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_suspensions[1])
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

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
void UAZBUS_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetSpringLength(FRONT_LEFT) << "  " << GetSpringLength(FRONT_RIGHT) << "  "
                 << GetSpringLength(REAR_LEFT) << "  " << GetSpringLength(REAR_RIGHT) << "\n";
        GetLog() << "Deformation [m]  " << GetSpringDeformation(FRONT_LEFT) << "  " << GetSpringDeformation(FRONT_RIGHT)
                 << "  " << GetSpringDeformation(REAR_LEFT) << "  " << GetSpringDeformation(REAR_RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetSpringForce(FRONT_LEFT) << "  " << GetSpringForce(FRONT_RIGHT) << "  "
                 << GetSpringForce(REAR_LEFT) << "  " << GetSpringForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetShockLength(FRONT_LEFT) << "  " << GetShockLength(FRONT_RIGHT) << "  "
                 << GetShockLength(REAR_LEFT) << "  " << GetShockLength(REAR_RIGHT) << "\n";
        GetLog() << "Velocity [m/s]   " << GetShockVelocity(FRONT_LEFT) << "  " << GetShockVelocity(FRONT_RIGHT) << "  "
                 << GetShockVelocity(REAR_LEFT) << "  " << GetShockVelocity(REAR_RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetShockForce(FRONT_LEFT) << "  " << GetShockForce(FRONT_RIGHT) << "  "
                 << GetShockForce(REAR_LEFT) << "  " << GetShockForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
