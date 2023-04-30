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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV 9-body vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "chrono_models/vehicle/hmmwv/brake/HMMWV_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/brake/HMMWV_BrakeShafts.h"
#include "chrono_models/vehicle/hmmwv/suspension/HMMWV_DoubleWishboneReduced.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_Driveline2WD.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_Driveline4WD.h"
#include "chrono_models/vehicle/hmmwv/driveline/HMMWV_SimpleDriveline.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArm.h"
#include "chrono_models/vehicle/hmmwv/steering/HMMWV_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleReduced::HMMWV_VehicleReduced(const bool fixed,
                                           DrivelineTypeWV drive_type,
                                           BrakeType brake_type,
                                           SteeringTypeWV steering_type,
                                           ChContactMethod contact_method,
                                           CollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVreduced", contact_method, drive_type) {
    Create(fixed, brake_type, steering_type, chassis_collision_type);
}

HMMWV_VehicleReduced::HMMWV_VehicleReduced(ChSystem* system,
                                           const bool fixed,
                                           DrivelineTypeWV drive_type,
                                           BrakeType brake_type,
                                           SteeringTypeWV steering_type,
                                           CollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVreduced", system, drive_type) {
    Create(fixed, brake_type, steering_type, chassis_collision_type);
}

void HMMWV_VehicleReduced::Create(bool fixed,
                                  BrakeType brake_type,
                                  SteeringTypeWV steering_type,
                                  CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<HMMWV_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    switch (steering_type) {
        case SteeringTypeWV::PITMAN_ARM:
            m_steerings[0] = chrono_types::make_shared<HMMWV_PitmanArm>("Steering");
            break;
        case SteeringTypeWV::RACK_PINION:
            m_steerings[0] = chrono_types::make_shared<HMMWV_RackPinion>("Steering");
            break;
        default:
            GetLog() << "Steering type NOT supported\n";
            break;
    }

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneReducedFront>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<HMMWV_DoubleWishboneReducedRear>("RearSusp");

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

HMMWV_VehicleReduced::~HMMWV_VehicleReduced() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = in2m * ChVector<>(56.735, 0, 3.174);
    m_steerings[0]->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], in2m * ChVector<>(66.59, 0, 1.039), ChVector<>(0), 0.0,
                           m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, in2m * ChVector<>(-66.4, 0, 1.039), ChVector<>(0), 0.0,
                           m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem.
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

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
