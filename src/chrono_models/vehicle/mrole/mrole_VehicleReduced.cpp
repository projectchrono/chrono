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
// mrole 9-body vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mrole/mrole_VehicleReduced.h"
#include "chrono_models/vehicle/mrole/mrole_Chassis.h"
#include "chrono_models/vehicle/mrole/mrole_BrakeSimple.h"
#include "chrono_models/vehicle/mrole/mrole_BrakeShafts.h"
#include "chrono_models/vehicle/mrole/mrole_DoubleWishboneReduced.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline2WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline4WD.h"
#include "chrono_models/vehicle/mrole/driveline/mrole_SimpleDriveline.h"
#include "chrono_models/vehicle/mrole/mrole_RackPinion.h"
#include "chrono_models/vehicle/mrole/mrole_Wheel.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_VehicleReduced::mrole_VehicleReduced(const bool fixed,
                                           DrivelineTypeWV drive_type,
                                           BrakeType brake_type,
                                           ChContactMethod contact_method,
                                           CollisionType chassis_collision_type)
    : mrole_Vehicle("mrolereduced", contact_method, drive_type) {
    Create(fixed, brake_type, chassis_collision_type);
}

mrole_VehicleReduced::mrole_VehicleReduced(ChSystem* system,
                                           const bool fixed,
                                           DrivelineTypeWV drive_type,
                                           BrakeType brake_type,
                                           CollisionType chassis_collision_type)
    : mrole_Vehicle("mrolereduced", system, drive_type) {
    Create(fixed, brake_type, chassis_collision_type);
}

void mrole_VehicleReduced::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<mrole_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<mrole_RackPinion1>("Steering");

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneReducedFront>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<mrole_DoubleWishboneReducedRear>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<mrole_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<mrole_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<mrole_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<mrole_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the driveline
    switch (m_driveType) {
        case DrivelineTypeWV::FWD:
        case DrivelineTypeWV::RWD:
            m_driveline = chrono_types::make_shared<mrole_Driveline2WD>("Driveline");
            break;
        default:
        case DrivelineTypeWV::AWD:
            m_driveline = chrono_types::make_shared<mrole_Driveline4WD>("Driveline");
            break;
        case DrivelineTypeWV::SIMPLE:
            m_driveline = chrono_types::make_shared<mrole_SimpleDriveline>("Driveline");
            break;
    }
}

mrole_VehicleReduced::~mrole_VehicleReduced() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void mrole_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
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

}  // end namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
