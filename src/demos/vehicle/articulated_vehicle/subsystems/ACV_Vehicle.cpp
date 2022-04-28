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
// Authors: Radu Serban
// =============================================================================
//
// Generic vehicle model with an articulated chassis.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "subsystems/ACV_Vehicle.h"
#include "subsystems/ACV_ChassisFront.h"
#include "subsystems/ACV_ChassisRear.h"
#include "subsystems/ACV_ChassisConnector.h"
#include "subsystems/ACV_RigidSuspension.h"
#include "subsystems/ACV_Wheel.h"
#include "subsystems/ACV_Driveline2WD.h"
#include "subsystems/ACV_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ACV_Vehicle::ACV_Vehicle(const bool fixed, ChContactMethod contactMethod)
    : ChWheeledVehicle("GenericWV", contactMethod) {
    // Create the front and rear chassis subsystems
    m_chassis = chrono_types::make_shared<ACV_ChassisFront>("ChassisFront", fixed);
    m_chassis_rear.resize(1);
    m_chassis_rear[0] = chrono_types::make_shared<ACV_ChassisRear>("ChassisRear");

    // Create the actuated articulation between front and rear chassis
    m_chassis_connectors.resize(1);
    m_chassis_connectors[0] = chrono_types::make_shared<ACV_ChassisConnector>("ChassisConnector");

    // Create the axle subsystems (suspension + wheels + brakes)
    m_axles.resize(2);

    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[0]->m_suspension = chrono_types::make_shared<ACV_RigidSuspension>("FrontSusp");
    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<ACV_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<ACV_Wheel>("Wheel_FR");
    m_axles[0]->m_brake_left = chrono_types::make_shared<ACV_BrakeSimple>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<ACV_BrakeSimple>("Brake_FR");

    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[1]->m_suspension = chrono_types::make_shared<ACV_RigidSuspension>("RearSusp");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<ACV_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<ACV_Wheel>("Wheel_RR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<ACV_BrakeSimple>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<ACV_BrakeSimple>("Brake_RR");

    // Create the driveline
    m_driveline = chrono_types::make_shared<ACV_Driveline2WD>("Driveline");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ACV_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);
    m_chassis_rear[0]->Initialize(m_chassis, WheeledCollisionFamily::CHASSIS);

    // Initialize the connection between front and rear chassis
    m_chassis_connectors[0]->Initialize(m_chassis, m_chassis_rear[0]);

    // Initialize the axle subsystems
    m_axles[0]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(0.5, 0, 0), ChVector<>(0, 0, 0.0), 0.0);
    m_axles[1]->Initialize(m_chassis_rear[0], nullptr, nullptr, ChVector<>(-0.5, 0, 0), ChVector<>(0, 0, 0), 0.0);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp = {0};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}
