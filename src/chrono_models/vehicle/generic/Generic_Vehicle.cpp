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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Generic 2-axle vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a generic rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/generic/Generic_Vehicle.h"
#include "chrono_models/vehicle/generic/Generic_Chassis.h"

#include "chrono_models/vehicle/generic/suspension/Generic_DoubleWishbone.h"
#include "chrono_models/vehicle/generic/suspension/Generic_DoubleWishboneReduced.h"
#include "chrono_models/vehicle/generic/suspension/Generic_SolidAxle.h"
#include "chrono_models/vehicle/generic/suspension/Generic_MultiLink.h"
#include "chrono_models/vehicle/generic/suspension/Generic_HendricksonPRIMAXX.h"
#include "chrono_models/vehicle/generic/suspension/Generic_MacPhersonStrut.h"

#include "chrono_models/vehicle/generic/steering/Generic_RackPinion.h"
#include "chrono_models/vehicle/generic/steering/Generic_PitmanArm.h"

#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/brake/Generic_BrakeSimple.h"
#include "chrono_models/vehicle/generic/brake/Generic_BrakeShafts.h"

#include "chrono_models/vehicle/generic/driveline/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/driveline/Generic_Driveline4WD.h"
#include "chrono_models/vehicle/generic/driveline/Generic_SimpleDriveline.h"

#include "chrono_models/vehicle/generic/Generic_AntirollBarRSD.h"

#include "chrono_models/vehicle/generic/tire/Generic_FialaTire.h"
#include "chrono_models/vehicle/generic/tire/Generic_Pac89Tire.h"
#include "chrono_models/vehicle/generic/tire/Generic_Pac02Tire.h"
#include "chrono_models/vehicle/generic/tire/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/tire/Generic_TMeasyTire.h"
#include "chrono_models/vehicle/generic/tire/Generic_TMsimpleTire.h"

#include "chrono_models/vehicle/generic/powertrain/Generic_EngineShafts.h"
#include "chrono_models/vehicle/generic/powertrain/Generic_EngineSimple.h"
#include "chrono_models/vehicle/generic/powertrain/Generic_EngineSimpleMap.h"
#include "chrono_models/vehicle/generic/powertrain/Generic_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/generic/powertrain/Generic_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------

Generic_Vehicle::Generic_Vehicle(const bool fixed,
                                 SuspensionTypeWV suspension_type,
                                 SteeringTypeWV steering_type,
                                 DrivelineTypeWV driveline_type,
                                 BrakeType brake_type,
                                 ChContactMethod contactMethod)
    : ChWheeledVehicle("GenericWV", contactMethod),
      m_suspension_type(suspension_type),
      m_steering_type(steering_type),
      m_driveline_type(driveline_type),
      m_brake_type(brake_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Generic_Chassis>("Chassis", fixed);

    // Create the axle subsystems (suspension + wheels + brakes)
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            m_axles[0]->m_suspension =
                chrono_types::make_shared<Generic_DoubleWishboneFront>("Front suspension", false);
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_DoubleWishboneRear>("Rear suspension", false);
            ////m_axles[0]->m_suspension = chrono_types::make_shared<Generic_DoubleWishbone>("Front suspension");
            ////m_axles[1]->m_suspension = chrono_types::make_shared<Generic_DoubleWishbone>("Rear suspension");
            break;
        case SuspensionTypeWV::SOLID_AXLE:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_SolidAxle>("Front Suspension");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_SolidAxle>("Rear Suspension");
            break;
        case SuspensionTypeWV::MULTI_LINK:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_MultiLink>("Front Suspension");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_MultiLink>("Rear Suspension");
            break;
        case SuspensionTypeWV::HENDRICKSON_PRIMAXX:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_HendricksonPRIMAXX>("Front suspension");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_HendricksonPRIMAXX>("Rear suspension");
            break;
        case SuspensionTypeWV::MACPHERSON_STRUT:
            m_axles[0]->m_suspension = chrono_types::make_shared<Generic_MacPhersonStrut>("Front suspension");
            m_axles[1]->m_suspension = chrono_types::make_shared<Generic_MacPhersonStrut>("Rear suspension");
            break;
        default:
            break;
    }

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_FR");

    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_RR");

    switch (m_brake_type) {
        case BrakeType::SIMPLE:
            m_axles[0]->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RR");
            break;
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<Generic_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<Generic_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<Generic_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Generic_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the antirollbar subsystem
    ////if (m_axles[0]->m_suspension->IsIndependent()) {
    ////  m_axles[0]->m_antirollbar = chrono_types::make_shared<Generic_AntirollBarRSD>("Antiroll Bar");
    ////}

    // Create the steering subsystem
    m_steerings.resize(1);
    switch (m_steering_type) {
        case SteeringTypeWV::PITMAN_ARM:
            m_steerings[0] = chrono_types::make_shared<Generic_PitmanArm>("Steering");
            break;
        case SteeringTypeWV::RACK_PINION:
            m_steerings[0] = chrono_types::make_shared<Generic_RackPinion>("Steering");
            break;
    }

    // Create the driveline
    switch (m_driveline_type) {
        case DrivelineTypeWV::AWD:
            m_driveline = chrono_types::make_shared<Generic_Driveline4WD>("Driveline");
            break;
        case DrivelineTypeWV::FWD:
        case DrivelineTypeWV::RWD:
            m_driveline = chrono_types::make_shared<Generic_Driveline2WD>("Driveline");
            break;
        case DrivelineTypeWV::SIMPLE:
            m_driveline = chrono_types::make_shared<Generic_SimpleDriveline>("Driveline");
            break;
    }
}

// -----------------------------------------------------------------------------

void Generic_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem
    // (specify the steering subsystem's frame relative to the chassis reference frame).
    ChVector<> offset;
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            offset = ChVector<>(1.24498, 0, 0.101322);
            break;
        case SuspensionTypeWV::SOLID_AXLE:
            offset = ChVector<>(2.1, 0, -0.02);
            break;
        case SuspensionTypeWV::MULTI_LINK:
            offset = ChVector<>(1.25, 0, 0.01);
            break;
        case SuspensionTypeWV::HENDRICKSON_PRIMAXX:
            offset = ChVector<>(1.25, 0, -0.21);
            break;
        case SuspensionTypeWV::MACPHERSON_STRUT:
            offset = ChVector<>(1.25, 0, 0.03);
            break;
        default:
            break;
    }
    m_steerings[0]->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the axle subsystems
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(1.688965, 0, 0), ChVector<>(1.3, 0, 0.0),
                           0.0);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(-1.688965, 0, 0), ChVector<>(1.3, 0, 0.0), 0.0);

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveline_type) {
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

// -----------------------------------------------------------------------------

double Generic_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetSpringForce(side);
        default:
            return -1;
    }
}

double Generic_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetSpringLength(side);
        default:
            return -1;
    }
}

double Generic_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
        default:
            return -1;
    }
}

double Generic_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockForce(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockForce(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetShockForce(side);
        default:
            return -1;
    }
}

double Generic_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockLength(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockLength(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetShockLength(side);
        default:
            return -1;
    }
}

double Generic_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            return std::static_pointer_cast<ChDoubleWishbone>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        case SuspensionTypeWV::SOLID_AXLE:
            return std::static_pointer_cast<ChSolidAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        case SuspensionTypeWV::MULTI_LINK:
            return std::static_pointer_cast<ChMultiLink>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        case SuspensionTypeWV::MACPHERSON_STRUT:
            return std::static_pointer_cast<ChMacPhersonStrut>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
        default:
            return -1;
    }
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Generic_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    switch (m_suspension_type) {
        case SuspensionTypeWV::DOUBLE_WISHBONE:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChDoubleWishbone>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChDoubleWishbone>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        case SuspensionTypeWV::SOLID_AXLE:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChSolidAxle>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        case SuspensionTypeWV::MULTI_LINK:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMultiLink>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        case SuspensionTypeWV::MACPHERSON_STRUT:
            GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMacPhersonStrut>(m_axles[0]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
            std::static_pointer_cast<ChMacPhersonStrut>(m_axles[1]->m_suspension)
                ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
            break;
        default:
            break;
    }

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
void Generic_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        GetLog() << "Deformation [inch]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        GetLog() << "Velocity [inch/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------

void Generic_Vehicle::CreateAndInitializeTires(TireModelType tire_type, VisualizationType vis_type) {
    switch (tire_type) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (tire_type == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<Generic_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<Generic_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<Generic_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<Generic_RigidTire>("RR", use_mesh);

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<Generic_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<Generic_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_FialaTire>("RR");

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<Generic_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<Generic_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_TMeasyTire>("RR");

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }

        case TireModelType::TMSIMPLE: {
            auto tire_FL = chrono_types::make_shared<Generic_TMsimpleTire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_TMsimpleTire>("FR");
            auto tire_RL = chrono_types::make_shared<Generic_TMsimpleTire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_TMsimpleTire>("RR");

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }

        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<Generic_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<Generic_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_Pac89Tire>("RR");

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }

        default:
            GetLog() << "Unsupported Tire Model Type! Switching to PAC02.\n";
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<Generic_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<Generic_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<Generic_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<Generic_Pac02Tire>("RR");

            InitializeTire(tire_FL, GetAxle(0)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_FR, GetAxle(0)->m_wheels[RIGHT], vis_type);
            InitializeTire(tire_RL, GetAxle(1)->m_wheels[LEFT], vis_type);
            InitializeTire(tire_RR, GetAxle(1)->m_wheels[RIGHT], vis_type);

            break;
        }
    }
}

void Generic_Vehicle::CreateAndInitializePowertrain(EngineModelType engine_type,
                                                    TransmissionModelType transmission_type) {
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;

    switch (engine_type) {
        default:
        case EngineModelType::SHAFTS:
            engine = chrono_types::make_shared<Generic_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<Generic_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<Generic_EngineSimple>("Engine");
            break;
    }
    switch (transmission_type) {
        default:
        case TransmissionModelType::AUTOMATIC_SHAFTS:
            transmission = chrono_types::make_shared<Generic_AutomaticTransmissionShafts>("Transmission");
            break;
        case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
            transmission = chrono_types::make_shared<Generic_AutomaticTransmissionSimpleMap>("Transmission");
            break;
    }
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    InitializePowertrain(powertrain);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
