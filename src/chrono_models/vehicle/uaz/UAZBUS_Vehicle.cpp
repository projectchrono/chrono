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
// UAZBUS vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Vehicle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_BrakeSimple.h"
#include "chrono_models/vehicle/uaz/UAZBUS_BrakeShafts.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Chassis.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Driveline4WD.h"
#include "chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_RotaryArm.h"
#include "chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Wheel.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Vehicle::UAZBUS_Vehicle(const bool fixed,
                               BrakeType brake_type,
                               SteeringTypeWV steering_model,
                               ChContactMethod contact_method,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("UAZBUS", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, steering_model, chassis_collision_type);
}

UAZBUS_Vehicle::UAZBUS_Vehicle(ChSystem* system,
                               const bool fixed,
                               BrakeType brake_type,
                               SteeringTypeWV steering_model,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("UAZBUS", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, steering_model, chassis_collision_type);
}

void UAZBUS_Vehicle::Create(bool fixed,
                            BrakeType brake_type,
                            SteeringTypeWV steering_model,
                            CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<UAZBUS_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<UAZBUS_RotaryArm>("Steering");

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<UAZBUS_ToeBarLeafspringAxle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<UAZBUS_LeafspringAxle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<UAZBUS_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<UAZBUS_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<UAZBUS_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<UAZBUS_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<UAZBUS_BrakeSimpleFront>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<UAZBUS_BrakeSimpleFront>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<UAZBUS_BrakeSimpleFront>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<UAZBUS_BrakeSimpleFront>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<UAZBUS_BrakeShaftsFront>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<UAZBUS_BrakeShaftsFront>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<UAZBUS_BrakeShaftsFront>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<UAZBUS_BrakeShaftsFront>("Brake_RR");
            break;
    }

    // Create the driveline
    m_driveline = chrono_types::make_shared<UAZBUS_Driveline4WD>("Driveline");
    ////m_driveline = chrono_types::make_shared<UAZBUS_SimpleDriveline>("Driveline");
}

UAZBUS_Vehicle::~UAZBUS_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = ChVector<>(0, 0, 0);
    ChQuaternion<> rotation = Q_from_AngAxis(0, ChVector<>(0, 1, 0));
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0, 0, 0), ChVector<>(0), 0.0,
                           m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-2.3, 0, 0), ChVector<>(0), 0.0, m_omega[2],
                           m_omega[3]);

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());
    driven_susp_indexes[0] = 1;
    driven_susp_indexes[1] = 1;
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double UAZBUS_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double UAZBUS_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double UAZBUS_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double UAZBUS_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double UAZBUS_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double UAZBUS_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void UAZBUS_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[1]->m_suspension)
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

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
