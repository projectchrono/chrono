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
// Authors: Radu Serban, Asher Elmquist, Jayne Henry
// =============================================================================
//
// ARTcar full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/artcar/ARTcar_Vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

ARTcar_Vehicle::ARTcar_Vehicle(const bool fixed, ChContactMethod contact_method, CollisionType chassis_collision_type)
    : ChWheeledVehicle("ARTcar", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, chassis_collision_type);
}

ARTcar_Vehicle::ARTcar_Vehicle(ChSystem* system, const bool fixed, CollisionType chassis_collision_type)
    : ChWheeledVehicle("ARTcar", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, chassis_collision_type);
}

void ARTcar_Vehicle::Create(bool fixed, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<ARTcar_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<ARTcar_DoubleWishboneFront>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<ARTcar_DoubleWishboneRear>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<ARTcar_WheelLeft>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<ARTcar_WheelLeft>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<ARTcar_WheelLeft>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<ARTcar_WheelLeft>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<ARTcar_BrakeShafts>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<ARTcar_BrakeShafts>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<ARTcar_BrakeShafts>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<ARTcar_BrakeShafts>("Brake_RR");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<ARTcar_PitmanArm>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<ARTcar_Driveline4WD>("Driveline");
}

ARTcar_Vehicle::~ARTcar_Vehicle() {}

void ARTcar_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // ORIGIN WILL BE LOCATED AT THE CENTER OF THE FRONT AXLE

    // Initialize the chassis subsystem.
    m_chassis->Initialize(this, chassisPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(0, 0, 0);
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(0), ChVector3d(0), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(0, 0, 0), ChVector3d(0), 0.0, m_omega[2],
                           m_omega[3]);

    // Initialize the driveline subsystem (4WD)
    std::vector<int> driven_susp_indexes = {0, 1};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

double ARTcar_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double ARTcar_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double ARTcar_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

double ARTcar_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double ARTcar_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double ARTcar_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void ARTcar_Vehicle::LogHardpointLocations() {
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
void ARTcar_Vehicle::DebugLog(int what) {
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

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono
