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

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "chrono_models/vehicle/hmmwv/brake/HMMWV_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/brake/HMMWV_BrakeShafts.h"
#include "chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishbone.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_Driveline2WD.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_Driveline4WD.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_SimpleDriveline.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArm.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArmShafts.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleFull::HMMWV_VehicleFull(const bool fixed,
                                     DrivelineTypeWV drive_type,
                                     BrakeType brake_type,
                                     SteeringTypeWV steering_type,
                                     bool use_tierod_bodies,
                                     bool rigid_steering_column,
                                     ChContactMethod contact_method,
                                     CollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVfull", contact_method, drive_type) {
    Create(fixed, brake_type, steering_type, use_tierod_bodies, rigid_steering_column, chassis_collision_type);
}

HMMWV_VehicleFull::HMMWV_VehicleFull(ChSystem* system,
                                     const bool fixed,
                                     DrivelineTypeWV drive_type,
                                     BrakeType brake_type,
                                     SteeringTypeWV steering_type,
                                     bool use_tierod_bodies,
                                     bool rigid_steering_column,
                                     CollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVfull", system, drive_type) {
    Create(fixed, brake_type, steering_type, use_tierod_bodies, rigid_steering_column, chassis_collision_type);
}

void HMMWV_VehicleFull::Create(bool fixed,
                               BrakeType brake_type,
                               SteeringTypeWV steering_type,
                               bool use_tierod_bodies,
                               bool rigid_steering_column,
                               CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<HMMWV_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    switch (steering_type) {
        case SteeringTypeWV::PITMAN_ARM:
            m_steerings[0] = chrono_types::make_shared<HMMWV_PitmanArm>("Steering");
            break;
        case SteeringTypeWV::PITMAN_ARM_SHAFTS:
            m_steerings[0] = chrono_types::make_shared<HMMWV_PitmanArmShafts>("Steering", rigid_steering_column);
            break;
        case SteeringTypeWV::RACK_PINION:
            m_steerings[0] = chrono_types::make_shared<HMMWV_RackPinion>("Steering");
            break;
        default:
            std::cout << "Steering type NOT supported\n";
            break;
    }

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneFront>("FrontSusp", use_tierod_bodies);
    m_axles[1]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneRear>("RearSusp", use_tierod_bodies);

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<HMMWV_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<HMMWV_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<HMMWV_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<HMMWV_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<HMMWV_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<HMMWV_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the driveline
    switch (m_driveType) {
        case DrivelineTypeWV::FWD:
        case DrivelineTypeWV::RWD:
            m_driveline = chrono_types::make_shared<HMMWV_Driveline2WD>("Driveline");
            break;
        default:
        case DrivelineTypeWV::AWD:
            m_driveline = chrono_types::make_shared<HMMWV_Driveline4WD>("Driveline");
            break;
        case DrivelineTypeWV::SIMPLE:
            m_driveline = chrono_types::make_shared<HMMWV_SimpleDriveline>("Driveline");
            break;
    }
}

HMMWV_VehicleFull::~HMMWV_VehicleFull() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleFull::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(this, chassisPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(1.24498, 0, 0.101322);
    ChQuaternion<> rotation = QuatFromAngleY(18.5 * CH_PI / 180);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(1.688965, 0, 0), ChVector3d(0), 0.0,
                           m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-1.688965, 0, 0), ChVector3d(0), 0.0, m_omega[2],
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
        default:
        case DrivelineTypeWV::AWD:
        case DrivelineTypeWV::SIMPLE:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            break;
    }

    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
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
    std::cout << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector3d(-37.78, 0, 30.77), true);

    std::cout << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector3d(-170.77, 0, 30.77), true);

    std::cout << "\n\n";
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
// -----------------------------------------------------------------------------
void HMMWV_VehicleFull::DebugLog(int what) {
    if (what & OUT_SPRINGS) {
        std::cout << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        std::cout << "Length [m]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                  << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        std::cout << "Deformation [m]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                  << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        std::cout << "Force [N]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                  << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        std::cout << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        std::cout << "Length [m]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                  << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        std::cout << "Velocity [m/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                  << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        std::cout << "Force [N]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                  << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
