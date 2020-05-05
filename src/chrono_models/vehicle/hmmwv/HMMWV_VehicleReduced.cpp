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

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleReduced::HMMWV_VehicleReduced(const bool fixed,
                                           DrivelineType drive_type,
                                           ChContactMethod contact_method,
                                           ChassisCollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVreduced", contact_method, drive_type) {
    Create(fixed, chassis_collision_type);
}

HMMWV_VehicleReduced::HMMWV_VehicleReduced(ChSystem* system,
                                           const bool fixed,
                                           DrivelineType drive_type,
                                           ChassisCollisionType chassis_collision_type)
    : HMMWV_Vehicle("HMMWVreduced", system, drive_type) {
    Create(fixed, chassis_collision_type);
}

void HMMWV_VehicleReduced::Create(bool fixed, ChassisCollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<HMMWV_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<HMMWV_RackPinion>("Steering");

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

HMMWV_VehicleReduced::~HMMWV_VehicleReduced() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = in2m * ChVector<>(56.735, 0, 3.174);
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis->GetBody(), in2m * ChVector<>(66.59, 0, 1.039), ChVector<>(0),
                           m_steerings[0]->GetSteeringLink(), 0, 0.0, m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis->GetBody(), in2m * ChVector<>(-66.4, 0, 1.039), ChVector<>(0),
                           m_chassis->GetBody(), -1, 0.0, m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem.
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

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
