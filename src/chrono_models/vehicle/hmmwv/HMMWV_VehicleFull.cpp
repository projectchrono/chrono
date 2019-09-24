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
// HMMWV full vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleFull::HMMWV_VehicleFull(const bool fixed,
                                     DrivelineType drive_type,
                                     SteeringType steering_type,
                                     bool rigid_steering_column,
                                     ChMaterialSurface::ContactMethod contact_method,
                                     ChassisCollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVfull", contact_method, drive_type) {
    Create(fixed, steering_type, rigid_steering_column, chassis_collision_type);
}

HMMWV_VehicleFull::HMMWV_VehicleFull(ChSystem* system,
                                     const bool fixed,
                                     DrivelineType drive_type,
                                     SteeringType steering_type,
                                     bool rigid_steering_column,
                                     ChassisCollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVfull", system, drive_type) {
    Create(fixed, steering_type, rigid_steering_column, chassis_collision_type);
}

void HMMWV_VehicleFull::Create(bool fixed,
                               SteeringType steering_type,
                               bool rigid_steering_column,
                               ChassisCollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<HMMWV_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    switch (steering_type) {
        case SteeringType::PITMAN_ARM:
            m_steerings[0] = chrono_types::make_shared<HMMWV_PitmanArm>("Steering");
            break;
        case SteeringType::PITMAN_ARM_SHAFTS:
            m_steerings[0] = chrono_types::make_shared<HMMWV_PitmanArmShafts>("Steering", rigid_steering_column);
            break;
        default:
            GetLog() << "Steering type NOT supported\n";
            break;
    }

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneFront>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneRear>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<HMMWV_WheelLeft>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<HMMWV_WheelRight>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<HMMWV_WheelLeft>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<HMMWV_WheelRight>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_RR");

    // Create the driveline
    switch (m_driveType) {
        case DrivelineType::FWD:
        case DrivelineType::RWD:
            m_driveline = chrono_types::make_shared<HMMWV_Driveline2WD>("Driveline");
            break;
        case DrivelineType::AWD:
            m_driveline = chrono_types::make_shared<HMMWV_Driveline4WD>("Driveline");
            break;
        case DrivelineType::SIMPLE:
            m_driveline = chrono_types::make_shared<HMMWV_SimpleDriveline>("Driveline");
            break;
    }
}

HMMWV_VehicleFull::~HMMWV_VehicleFull() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleFull::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = ChVector<>(1.24498, 0, 0.101322);
    ChQuaternion<> rotation = Q_from_AngAxis(18.5 * CH_C_PI / 180, ChVector<>(0, 1, 0));
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.688965, 0, 0), ChVector<>(0),
                           m_steerings[0]->GetSteeringLink(), 0, 0.0, m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis->GetBody(), ChVector<>(-1.688965, 0, 0), ChVector<>(0), m_chassis->GetBody(), -1,
                           0.0, m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveType) {
        case DrivelineType::FWD:
            driven_susp_indexes[0] = 0;
            break;
        case DrivelineType::RWD:
            driven_susp_indexes[0] = 1;
            break;
        case DrivelineType::AWD:
        case DrivelineType::SIMPLE:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            break;
    }

    m_driveline->Initialize(m_chassis->GetBody(), m_axles, driven_susp_indexes);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_VehicleFull::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double HMMWV_VehicleFull::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double HMMWV_VehicleFull::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_VehicleFull::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double HMMWV_VehicleFull::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double HMMWV_VehicleFull::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void HMMWV_VehicleFull::LogHardpointLocations() {
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
void HMMWV_VehicleFull::DebugLog(int what) {
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

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
