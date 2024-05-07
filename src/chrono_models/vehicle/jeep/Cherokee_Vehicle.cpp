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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Base class for modeling an entire Jeep Cherokee 1997 vehicle assembly
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_Vehicle.h"

#include "chrono_models/vehicle/jeep/Cherokee_BrakeShafts.h"
#include "chrono_models/vehicle/jeep/Cherokee_Chassis.h"
#include "chrono_models/vehicle/jeep/Cherokee_SolidAxleFront.h"
#include "chrono_models/vehicle/jeep/Cherokee_Driveline4WD.h"
#include "chrono_models/vehicle/jeep/Cherokee_SolidAxleRear.h"
#include "chrono_models/vehicle/jeep/Cherokee_Steering.h"
#include "chrono_models/vehicle/jeep/Cherokee_Wheel.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_Vehicle::Cherokee_Vehicle(const bool fixed,
                                   BrakeType brake_type,
                                   ChContactMethod contact_method,
                                   CollisionType chassis_collision_type)
    : ChWheeledVehicle("Cherokee", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

Cherokee_Vehicle::Cherokee_Vehicle(ChSystem* system,
                                   const bool fixed,
                                   BrakeType brake_type,
                                   CollisionType chassis_collision_type)
    : ChWheeledVehicle("Cherokee", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

void Cherokee_Vehicle::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Cherokee_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Cherokee_SolidAxleFront>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Cherokee_SolidAxleRear>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Cherokee_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Cherokee_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Cherokee_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Cherokee_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            /*
                        m_axles[0]->m_brake_left = chrono_types::make_shared<Sedan_BrakeSimple>("Brake_FL");
                        m_axles[0]->m_brake_right = chrono_types::make_shared<Sedan_BrakeSimple>("Brake_FR");
                        m_axles[1]->m_brake_left = chrono_types::make_shared<Sedan_BrakeSimple>("Brake_RL");
                        m_axles[1]->m_brake_right = chrono_types::make_shared<Sedan_BrakeSimple>("Brake_RR");
                        break;
            */
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<Cherokee_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<Cherokee_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<Cherokee_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Cherokee_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the antirollbar systems
    // m_axles[0]->m_antirollbar = chrono_types::make_shared<BMW_E90_AntiRollBarFront>("AntirollBarFront");
    // m_axles[1]->m_antirollbar = chrono_types::make_shared<BMW_E90_AntiRollBarRear>("AntirollBarRear");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Cherokee_Steering>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<Cherokee_Driveline4WD>("Driveline");
}

Cherokee_Vehicle::~Cherokee_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Cherokee_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(0.0, 0.0, 0.0);
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(0), ChVector3d(-0.2, 0, 0.1), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-2.578, 0, 0), ChVector3d(-2.45717, 0, 0), 0.0,
                           m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem (AWD)
    std::vector<int> driven_susp_indexes = {0, 1};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Cherokee_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    if(axle == 0)
        return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double Cherokee_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    if(axle == 0)
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double Cherokee_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    if(axle == 0)
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Cherokee_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    if(axle == 0)
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double Cherokee_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    if(axle == 0)
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double Cherokee_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    if(axle == 0)
    return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
    else
        return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Cherokee_Vehicle::LogHardpointLocations() {
    std::cout << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n\n";
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void Cherokee_Vehicle::DebugLog(int what) {
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

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono
