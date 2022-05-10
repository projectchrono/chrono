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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MTV full vehicle model
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/MTV_Vehicle.h"
#include "chrono_models/vehicle/mtv/MTV_ChassisRear.h"
#include "chrono_models/vehicle/mtv/MTV_Balancer.h"
#include "chrono_models/vehicle/mtv/MTV_LeafspringAxle1.h"
#include "chrono_models/vehicle/mtv/MTV_LeafspringAxle2.h"
#include "chrono_models/vehicle/mtv/MTV_Solid3LinkAxle1.h"
#include "chrono_models/vehicle/mtv/MTV_Solid3LinkAxle2.h"

#include "chrono_models/vehicle/mtv/FMTV_ChassisFront.h"
#include "chrono_models/vehicle/mtv/FMTV_BrakeSimple.h"
#include "chrono_models/vehicle/mtv/FMTV_BrakeShafts.h"
#include "chrono_models/vehicle/mtv/FMTV_Driveline4WD.h"
#include "chrono_models/vehicle/mtv/FMTV_AntiRollBar.h"
#include "chrono_models/vehicle/mtv/FMTV_RotaryArm.h"
#include "chrono_models/vehicle/mtv/FMTV_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/mtv/FMTV_ToebarLeafspringAxle.h"
#include "chrono_models/vehicle/mtv/FMTV_Wheel.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MTV_Vehicle::MTV_Vehicle(const bool fixed,
                         bool use_walking_beam,
                         BrakeType brake_type,
                         ChContactMethod contact_method,
                         CollisionType chassis_collision_type)
    : ChWheeledVehicle("MTV", contact_method), m_omega({0, 0, 0, 0, 0, 0}) {
    Create(fixed, use_walking_beam, brake_type, chassis_collision_type);
}

MTV_Vehicle::MTV_Vehicle(ChSystem* system,
                         const bool fixed,
                         bool use_walking_beam,
                         BrakeType brake_type,
                         CollisionType chassis_collision_type)
    : ChWheeledVehicle("MTV", system), m_omega({0, 0, 0, 0, 0, 0}) {
    Create(fixed, use_walking_beam, brake_type, chassis_collision_type);
}

void MTV_Vehicle::Create(bool fixed,
                         bool use_walking_beam,
                         BrakeType brake_type,
                         CollisionType chassis_collision_type) {
    // Create the front and rear chassis subsystems
    m_chassis = chrono_types::make_shared<FMTV_ChassisFront>("ChassisFront", fixed, chassis_collision_type);
    m_chassis_rear.resize(1);
    m_chassis_rear[0] = chrono_types::make_shared<MTV_ChassisRear>("ChassisRear");

    // Create the torsion articulation between front and rear chassis
    m_chassis_connectors.resize(1);
    m_chassis_connectors[0] = chrono_types::make_shared<MTV_ChassisConnector>("ChassisConnector");

    // Create the balancer subsystem
    m_subchassis.resize(1);
    m_subchassis[0] = chrono_types::make_shared<MTV_Balancer>("Balancer");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<FMTV_RotaryArm>("Steering");

    // Create the axle subsystems
    m_axles.resize(3);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<FMTV_ToebarLeafspringAxle>("FrontSusp");
    if (use_walking_beam) {
        m_axles[1]->m_suspension = chrono_types::make_shared<MTV_Solid3LinkAxle1>("RearSusp1");
        m_axles[2]->m_suspension = chrono_types::make_shared<MTV_Solid3LinkAxle2>("RearSusp2");
    } else {
        m_axles[1]->m_suspension = chrono_types::make_shared<MTV_LeafspringAxle1>("RearSusp1");
        m_axles[2]->m_suspension = chrono_types::make_shared<MTV_LeafspringAxle2>("RearSusp2");    
    }

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<FMTV_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<FMTV_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<FMTV_Wheel>("Wheel_RL1");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<FMTV_Wheel>("Wheel_RR1");
    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<FMTV_Wheel>("Wheel_RL2");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<FMTV_Wheel>("Wheel_RR2");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_RL1");
            m_axles[1]->m_brake_right = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_RR1");
            m_axles[2]->m_brake_left = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_RL2");
            m_axles[2]->m_brake_right = chrono_types::make_shared<FMTV_BrakeSimple>("Brake_RR2");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_RL1");
            m_axles[1]->m_brake_right = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_RR1");
            m_axles[2]->m_brake_left = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_RL2");
            m_axles[2]->m_brake_right = chrono_types::make_shared<FMTV_BrakeShafts>("Brake_RR2");
            break;
    }

    // Create the antirollbar system
    // m_axles[1]->m_antirollbar = chrono_types::make_shared<FMTV_AntirollBarRSD>("AntirollBar");

    // Create the driveline
    m_driveline = chrono_types::make_shared<FMTV_Driveline4WD>("Driveline");
    ////m_driveline = chrono_types::make_shared<FMTV_SimpleDriveline>("Driveline");
}

MTV_Vehicle::~MTV_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MTV_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystems.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);
    m_chassis_rear[0]->Initialize(m_chassis, WheeledCollisionFamily::CHASSIS);

    // Initialize the connection between front and rear chassis
    m_chassis_connectors[0]->Initialize(m_chassis, m_chassis_rear[0]);

    // Initialize the balancer subsystem
    m_subchassis[0]->Initialize(m_chassis_rear[0], ChVector<>(-4.1, 0.0, 0.26));

    // Initialize the steering subsystem (specify the steering frame relative to the chassis reference frame)
    ChVector<> offset = ChVector<>(0, 0, 0);
    ChQuaternion<> rotation = Q_from_AngY(0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0, 0, 0), ChVector<>(0), 0.0, m_omega[0],
                           m_omega[1]);

    // rear axles mounted on rear chassis and subchassis
    m_axles[1]->Initialize(m_chassis_rear[0], m_subchassis[0], nullptr, ChVector<>(-3.4, 0, 0), ChVector<>(0), 0.0,
                           m_omega[2], m_omega[3]);
    m_axles[2]->Initialize(m_chassis_rear[0], m_subchassis[0], nullptr, ChVector<>(-4.8, 0, 0), ChVector<>(0), 0.0,
                           m_omega[4], m_omega[5]);

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    driven_susp_indexes[0] = 1;
    driven_susp_indexes[1] = 2;

    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double MTV_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double MTV_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double MTV_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double MTV_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double MTV_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double MTV_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void MTV_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarLeafspringAxle>(m_axles[1]->m_suspension)
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
void MTV_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        GetLog() << "Deformation [m]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        GetLog() << "Velocity [m/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
