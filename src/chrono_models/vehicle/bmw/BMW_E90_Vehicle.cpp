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
// Base class for the BMW E90 vehicle models
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// Suspension/ARB parameters fit to test results presented in SAE Paper 2007-01-0817
// The chassis roll angle = 2.0 deg at steady state lateral acceleration = 0.6 g
//
// Steady state cornering simulation shows that the vehicle is understeering, there
// is no real test available, but this result was expected for a passenger car.
//
// The steering geometry has been changed for chrono, because the original data
// lead to a situation where the inner wheel turn angle is smaller than the outer
// one at cornering. The steering trapez has been recalculated to standard behavior.
//
// The tire parameters where calculated from data with a Matlab script, both presented
// in SAE Paper 2007-01-0818. Actually no alignment torque, camber influence and
// relaxation is considered.
//
// SAE 2007-01-0817 shows a test called  'Slowly Increasing Steer', where the vehicle
// runs with constant speed of 50 mph (80 km/h) and the steering wheel angle is increased
// until the friction limit is reached. To reproduce a similar result with the
// cornering test the road friction coefficient must be set to 1.0.
//
// The wall-to-wall test shows a minimal distance of 11.4m.
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/bmw/BMW_E90_Vehicle.h"
#include "chrono_models/vehicle/bmw/BMW_E90_BrakeShafts.h"
#include "chrono_models/vehicle/bmw/BMW_E90_Chassis.h"
#include "chrono_models/vehicle/bmw/BMW_E90_MacPhersonStrut.h"
#include "chrono_models/vehicle/bmw/BMW_E90_AntiRollBarFront.h"
#include "chrono_models/vehicle/bmw/BMW_E90_AntiRollBarRear.h"
#include "chrono_models/vehicle/bmw/BMW_E90_Driveline.h"
#include "chrono_models/vehicle/bmw/BMW_E90_DoubleWishbone.h"
#include "chrono_models/vehicle/bmw/BMW_E90_Steering.h"
#include "chrono_models/vehicle/bmw/BMW_E90_FrontWheel.h"
#include "chrono_models/vehicle/bmw/BMW_E90_RearWheel.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_Vehicle::BMW_E90_Vehicle(const bool fixed,
                                 BrakeType brake_type,
                                 ChContactMethod contact_method,
                                 CollisionType chassis_collision_type)
    : ChWheeledVehicle("BMW_E90", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

BMW_E90_Vehicle::BMW_E90_Vehicle(ChSystem* system,
                                 const bool fixed,
                                 BrakeType brake_type,
                                 CollisionType chassis_collision_type)
    : ChWheeledVehicle("BMW_E90", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

void BMW_E90_Vehicle::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<BMW_E90_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<BMW_E90_MacPhersonStrut>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<BMW_E90_DoubleWishbone>("RearSusp", true);

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<BMW_E90_FrontWheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<BMW_E90_FrontWheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<BMW_E90_RearWheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<BMW_E90_RearWheel>("Wheel_RR");

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
            m_axles[0]->m_brake_left = chrono_types::make_shared<BMW_E90_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<BMW_E90_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<BMW_E90_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<BMW_E90_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the antirollbar systems
    m_axles[0]->m_antirollbar = chrono_types::make_shared<BMW_E90_AntiRollBarFront>("AntirollBarFront");
    m_axles[1]->m_antirollbar = chrono_types::make_shared<BMW_E90_AntiRollBarRear>("AntirollBarRear");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<BMW_E90_Steering>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<BMW_E90_Driveline>("Driveline");
}

BMW_E90_Vehicle::~BMW_E90_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void BMW_E90_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(this, chassisPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(-0.2, 0.0, -0.05);
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(0), ChVector3d(-0.2, 0, 0.1), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-2.75717, 0, 0), ChVector3d(-2.45717, 0, 0), 0.0,
                           m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem (FWD)
    std::vector<int> driven_susp_indexes = {1};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double BMW_E90_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double BMW_E90_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double BMW_E90_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double BMW_E90_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double BMW_E90_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double BMW_E90_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void BMW_E90_Vehicle::LogHardpointLocations() {
    std::cout << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_axles[1]->m_suspension)
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
void BMW_E90_Vehicle::DebugLog(int what) {
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

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
