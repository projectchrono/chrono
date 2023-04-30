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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// mrole full vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mrole/mrole_VehicleFull.h"
#include "chrono_models/vehicle/mrole/mrole_Chassis.h"
#include "chrono_models/vehicle/mrole/mrole_BrakeSimple.h"
#include "chrono_models/vehicle/mrole/mrole_BrakeShafts.h"
#include "chrono_models/vehicle/mrole/mrole_DoubleWishbone.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline2WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline4WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline6WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline8WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_SimpleDriveline.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_SimpleDrivelineXWD.h"
#include "chrono_models/vehicle/mrole/mrole_RackPinion.h"
#include "chrono_models/vehicle/mrole/mrole_Wheel.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_VehicleFull::mrole_VehicleFull(const bool fixed,
                                     DrivelineTypeWV drive_type,
                                     BrakeType brake_type,
                                     SteeringTypeWV steering_type,
                                     bool rigid_steering_column,
                                     ChContactMethod contact_method,
                                     CollisionType chassis_collision_type)
    : mrole_Vehicle("mrolefull", contact_method, drive_type) {
    Create(fixed, brake_type, steering_type, rigid_steering_column, chassis_collision_type);
}

mrole_VehicleFull::mrole_VehicleFull(ChSystem* system,
                                     const bool fixed,
                                     DrivelineTypeWV drive_type,
                                     BrakeType brake_type,
                                     SteeringTypeWV steering_type,
                                     bool rigid_steering_column,
                                     CollisionType chassis_collision_type)
    : mrole_Vehicle("mrolefull", system, drive_type) {
    Create(fixed, brake_type, steering_type, rigid_steering_column, chassis_collision_type);
}

void mrole_VehicleFull::Create(bool fixed,
                               BrakeType brake_type,
                               SteeringTypeWV steering_type,
                               bool rigid_steering_column,
                               CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<mrole_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(2);
    m_steerings[0] = chrono_types::make_shared<mrole_RackPinion1>("Steering1");
    m_steerings[1] = chrono_types::make_shared<mrole_RackPinion2>("Steering2");

    // Create the axle subsystems
    m_axles.resize(4);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();
    m_axles[3] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneFront>("FrontSusp1");
    m_axles[1]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneFront>("FrontSusp2");
    m_axles[2]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneRear>("RearSusp1");
    m_axles[3]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneRear>("RearSusp2");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_FL1");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_FR1");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_FR2");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_FR2");
    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_RL1");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_RR1");
    m_axles[3]->m_wheels.resize(2);
    m_axles[3]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_RL2");
    m_axles[3]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_RR2");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FL1");
            m_axles[0]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FR1");
            m_axles[1]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FL2");
            m_axles[1]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FR2");
            m_axles[2]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RL1");
            m_axles[2]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RR1");
            m_axles[3]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RL2");
            m_axles[3]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RR2");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FL1");
            m_axles[0]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FR1");
            m_axles[1]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FL2");
            m_axles[1]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FR2");
            m_axles[2]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RL1");
            m_axles[2]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RR1");
            m_axles[3]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RL2");
            m_axles[3]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RR2");
            break;
    }

    // Create the driveline
    switch (m_driveType) {
        case DrivelineTypeWV::FWD:
        case DrivelineTypeWV::RWD:
            m_driveline = chrono_types::make_shared<mrole_Driveline2WD>("Driveline");
            break;
        case DrivelineTypeWV::AWD:
            m_driveline = chrono_types::make_shared<mrole_Driveline4WD>("Driveline");
            break;
        case DrivelineTypeWV::AWD6:
            m_driveline = chrono_types::make_shared<mrole_Driveline6WD>("Driveline");
            break;
        case DrivelineTypeWV::AWD8:
            m_driveline = chrono_types::make_shared<mrole_Driveline8WD>("Driveline");
            break;
        case DrivelineTypeWV::SIMPLE:
            m_driveline = chrono_types::make_shared<mrole_SimpleDriveline>("Driveline");
            break;
        case DrivelineTypeWV::SIMPLE_XWD:
            m_driveline = chrono_types::make_shared<mrole_SimpleDrivelineXWD>("Driveline");
            break;
    }
}

mrole_VehicleFull::~mrole_VehicleFull() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_VehicleFull::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset1 = ChVector<>(-0.45, 0, 0.0);
    ChVector<> offset2 = ChVector<>(-0.45 - 1.55, 0, 0.0);
    m_steerings[0]->Initialize(m_chassis, offset1, QUNIT);
    m_steerings[1]->Initialize(m_chassis, offset2, QUNIT);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0.0, 0, 0), ChVector<>(0), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, m_steerings[1], ChVector<>(-1.55, 0, 0), ChVector<>(0), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[2]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-3.45, 0, 0), ChVector<>(0), 0.0, m_omega[2],
                           m_omega[3]);
    m_axles[3]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-5.0, 0, 0), ChVector<>(0), 0.0, m_omega[2],
                           m_omega[3]);

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveType) {
        case DrivelineTypeWV::FWD:
            driven_susp_indexes[0] = 0;
            break;
        case DrivelineTypeWV::RWD:
            driven_susp_indexes[0] = 1;
            break;
        case DrivelineTypeWV::AWD:
        case DrivelineTypeWV::SIMPLE:
            driven_susp_indexes[0] = 2;
            driven_susp_indexes[1] = 3;
            break;
        case DrivelineTypeWV::AWD6:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 2;
            driven_susp_indexes[2] = 3;
            break;
        case DrivelineTypeWV::AWD8:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            driven_susp_indexes[2] = 2;
            driven_susp_indexes[3] = 3;
            break;
        case DrivelineTypeWV::SIMPLE_XWD:
            driven_susp_indexes.resize(4);
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            driven_susp_indexes[2] = 2;
            driven_susp_indexes[3] = 3;
            break;
    }

    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double mrole_VehicleFull::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double mrole_VehicleFull::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double mrole_VehicleFull::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double mrole_VehicleFull::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double mrole_VehicleFull::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double mrole_VehicleFull::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void mrole_VehicleFull::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(-170.77, 0, 30.77), true);

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
// -----------------------------------------------------------------------------
void mrole_VehicleFull::DebugLog(int what) {
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

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
