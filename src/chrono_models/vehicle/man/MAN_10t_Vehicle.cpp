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
// MAN10t full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/man/MAN_10t_Vehicle.h"
#include "chrono_models/vehicle/man/MAN_10t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeShafts.h"
#include "chrono_models/vehicle/man/suspension/MAN_7t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/suspension/MAN_10t_Front1Axle.h"
#include "chrono_models/vehicle/man/suspension/MAN_10t_Front2Axle.h"
#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"
#include "chrono_models/vehicle/man/MAN_5t_Driveline4WD.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleDrivelineXWD.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------

MAN_10t_Vehicle::MAN_10t_Vehicle(const bool fixed,
                                 BrakeType brake_type,
                                 ChContactMethod contact_method,
                                 CollisionType chassis_collision_type,
                                 bool use_8WD_drivetrain)
    : ChWheeledVehicle("MAN_10t", contact_method),
      m_omega({0, 0, 0, 0, 0, 0, 0, 0}),
      m_use_8WD_drivetrain(use_8WD_drivetrain) {
    Create(fixed, brake_type, chassis_collision_type);
}

MAN_10t_Vehicle::MAN_10t_Vehicle(ChSystem* system,
                                 const bool fixed,
                                 BrakeType brake_type,
                                 CollisionType chassis_collision_type,
                                 bool use_8WD_drivetrain)
    : ChWheeledVehicle("MAN_10t", system), m_omega({0, 0, 0, 0, 0, 0, 0, 0}), m_use_8WD_drivetrain(use_8WD_drivetrain) {
    Create(fixed, brake_type, chassis_collision_type);
}

void MAN_10t_Vehicle::Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<MAN_10t_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems (suspension + wheels + brakes)
    m_axles.resize(4);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();
    m_axles[3] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<MAN_10t_Front1Axle>("FrontSusp1");
    m_axles[1]->m_suspension = chrono_types::make_shared<MAN_10t_Front2Axle>("FrontSusp2");
    m_axles[2]->m_suspension = chrono_types::make_shared<MAN_7t_Solid3LinkAxle>("RearSusp1");
    m_axles[3]->m_suspension = chrono_types::make_shared<MAN_7t_Solid3LinkAxle>("RearSusp2");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FL1");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FR1");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FL2");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_FR2");

    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RL1");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RR1");

    m_axles[3]->m_wheels.resize(2);
    m_axles[3]->m_wheels[0] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RL2");
    m_axles[3]->m_wheels[1] = chrono_types::make_shared<MAN_5t_Wheel>("Wheel_RR2");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FL1");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FR1");

            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FL2");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_FR2");

            m_axles[2]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RL1");
            m_axles[2]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RR1");

            m_axles[3]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RL2");
            m_axles[3]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeSimple>("Brake_RR2");

            break;

        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FL1");
            m_axles[0]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FR1");

            m_axles[1]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FL2");
            m_axles[1]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_FR2");

            m_axles[2]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RL1");
            m_axles[2]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RR1");

            m_axles[3]->m_brake_left = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RL2");
            m_axles[3]->m_brake_right = chrono_types::make_shared<MAN_5t_BrakeShafts>("Brake_RR2");

            break;
    }

    // Create the steering subsystem
    m_steerings.resize(2);
    m_steerings[0] = chrono_types::make_shared<MAN_5t_RotaryArm>("Steering1");
    m_steerings[1] = chrono_types::make_shared<MAN_10t_RotaryArm2>("Steering2");

    // Create the driveline
    if (m_use_8WD_drivetrain) {
        m_driveline = chrono_types::make_shared<MAN_5t_SimpleDrivelineXWD>("Driveline");
    } else {
        m_driveline = chrono_types::make_shared<MAN_5t_Driveline4WD>("Driveline");
    }
}

MAN_10t_Vehicle::~MAN_10t_Vehicle() {}

// -----------------------------------------------------------------------------

void MAN_10t_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset1 = ChVector<>(0, 0, 0.0);      // 0.4 0 0.4
    ChVector<> offset2 = ChVector<>(-1.93, 0, 0.0);  // 0.4 0 0.4
    ChVector<> offset3 = ChVector<>(-5.6, 0, 0.0);   // 0.4 0 0.4
    ChVector<> offset4 = ChVector<>(-7.0, 0, 0.0);   // 0.4 0 0.4
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset1, rotation);
    m_steerings[1]->Initialize(m_chassis, offset2, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], offset1, ChVector<>(0), 0.0, m_omega[0], m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, m_steerings[1], offset2, ChVector<>(0), 0.0, m_omega[2], m_omega[3]);
    const double twin_tire_dist = 0.0;  // single tires only
    m_axles[2]->Initialize(m_chassis, nullptr, nullptr, offset3, ChVector<>(0), twin_tire_dist, m_omega[4], m_omega[5]);
    m_axles[3]->Initialize(m_chassis, nullptr, nullptr, offset4, ChVector<>(0), twin_tire_dist, m_omega[6], m_omega[7]);

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp_indexes;
    if (m_use_8WD_drivetrain) {
        driven_susp_indexes.resize(4);
        driven_susp_indexes[0] = 0;
        driven_susp_indexes[1] = 1;
        driven_susp_indexes[2] = 2;
        driven_susp_indexes[3] = 3;
        m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);
    } else {
        driven_susp_indexes.resize(2);
        driven_susp_indexes[0] = 2;
        driven_susp_indexes[1] = 3;
        m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);
    }

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void MAN_10t_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidBellcrankThreeLinkAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- FRONT suspension 2 hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidBellcrankThreeLinkAxle>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension 1 hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidThreeLinkAxle>(m_axles[2]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension 2 hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidThreeLinkAxle>(m_axles[3]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void MAN_10t_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS || what & OUT_SHOCKS) {
        GetLog() << "\n---- Spring and Shock information\n\n";
        for (int axle = 0; axle < 2; axle++) {
            std::string axlePosition = (axle == 0) ? "Front" : "Rear ";
            for (int side = LEFT; side <= RIGHT; side++) {
                for (auto& forceTSDA :
                     m_axles[axle]->m_suspension->ReportSuspensionForce(static_cast<VehicleSide>(side))) {
                    GetLog() << axlePosition << " " << (side == LEFT ? "Left " : "Right") << " ";
                    GetLog() << forceTSDA.name << std::string(10 - std::max(0, (int)forceTSDA.name.size()), ' ')
                             << " Length: " << forceTSDA.length << " m, Force: " << forceTSDA.force << " N\n";
                }
            }
        }
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
