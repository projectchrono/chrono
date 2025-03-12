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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He, Rainer Gericke
// =============================================================================
//
// MAN 5t full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/man/MAN_5t_Vehicle.h"
#include "chrono_models/vehicle/man/MAN_5t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeShafts.h"
#include "chrono_models/vehicle/man/suspension/MAN_5t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/suspension/MAN_5t_BellcrankSolid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_5t_Driveline4WD.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleDriveline.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------

MAN_5t_Vehicle::MAN_5t_Vehicle(const bool fixed,
                               BrakeType brake_type,
                               ChContactMethod contact_method,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("MAN_5t", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

MAN_5t_Vehicle::MAN_5t_Vehicle(ChSystem* system,
                               const bool fixed,
                               BrakeType brake_type,
                               CollisionType chassis_collision_type)
    : ChWheeledVehicle("MAN_5t", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, chassis_collision_type);
}

void MAN_5t_Vehicle::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<MAN_5t_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems (suspension + wheels + brakes)
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<MAN_5t_BellcrankSolid3LinkAxle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<MAN_5t_Solid3LinkAxle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<MAN_5t_RotaryArm>("Steering");

    // Create the driveline
    m_driveline = chrono_types::make_shared<MAN_5t_Driveline4WD>("Driveline");
}

MAN_5t_Vehicle::~MAN_5t_Vehicle() {}

// -----------------------------------------------------------------------------

void MAN_5t_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(this, chassisPos, chassisFwdVel, VehicleCollisionFamily::CHASSIS_FAMILY);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(0, 0, 0.0);  // 0.4 0 0.4
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(0, 0, 0), ChVector3d(0), 0.0, m_omega[0],
                           m_omega[1]);
    const double twin_tire_dist = 0.0;  // Michelin for 305/85 R22.5
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-4.5, 0, 0), ChVector3d(0), twin_tire_dist,
                           m_omega[2], m_omega[3]);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp_indexes;
    driven_susp_indexes.resize(2);
    driven_susp_indexes[0] = 0;
    driven_susp_indexes[1] = 1;
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void MAN_5t_Vehicle::LogHardpointLocations() {
    std::cout << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidBellcrankThreeLinkAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidThreeLinkAxle>(m_axles[1]->m_suspension)
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
void MAN_5t_Vehicle::DebugLog(int what) {
    if (what & OUT_SPRINGS || what & OUT_SHOCKS) {
        std::cout << "\n---- Spring and Shock information\n\n";
        for (int axle = 0; axle < 2; axle++) {
            std::string axlePosition = (axle == 0) ? "Front" : "Rear ";
            for (int side = LEFT; side <= RIGHT; side++) {
                for (auto& forceTSDA :
                     m_axles[axle]->m_suspension->ReportSuspensionForce(static_cast<VehicleSide>(side))) {
                    std::cout << axlePosition << " " << (side == LEFT ? "Left " : "Right") << " ";
                    std::cout << forceTSDA.name << std::string(10 - std::max(0, (int)forceTSDA.name.size()), ' ')
                              << " Length: " << forceTSDA.length << " m, Force: " << forceTSDA.force << " N\n";
                }
            }
        }
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
