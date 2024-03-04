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
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_TMsimpleTire.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimpleMap.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimple.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.h"
#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
HMMWV::HMMWV()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_driveType(DrivelineTypeWV::AWD),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::HMMWV(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_driveType(DrivelineTypeWV::AWD),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::~HMMWV() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void HMMWV::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void HMMWV::Initialize() {
    // Create and initialize the HMMWV vehicle
    m_vehicle = CreateVehicle();
    m_vehicle->SetCollisionSystemType(m_collsysType);
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;
    switch (m_engineType) {
        case EngineModelType::SHAFTS:
            engine = chrono_types::make_shared<HMMWV_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<HMMWV_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<HMMWV_EngineSimple>("Engine");
            break;
    }

    switch (m_transmissionType) {
        case TransmissionModelType::AUTOMATIC_SHAFTS:
            transmission = chrono_types::make_shared<HMMWV_AutomaticTransmissionShafts>("Transmission");
            break;
        case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
            transmission = chrono_types::make_shared<HMMWV_AutomaticTransmissionSimpleMap>("Transmission");
            break;
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<HMMWV_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_FialaTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<HMMWV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
            
        default:
            GetLog() << "Unsupported Tire Model Type!, Switching to TMsimple.\n";
        case TireModelType::TMSIMPLE: {
            auto tire_FL = chrono_types::make_shared<HMMWV_TMsimpleTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_TMsimpleTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_TMsimpleTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_TMsimpleTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac89Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac02Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
            
        case TireModelType::ANCF: {
            auto tire_FL = chrono_types::make_shared<HMMWV_ANCFTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_ANCFTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_ANCFTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_ANCFTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::REISSNER: {
            auto tire_FL = chrono_types::make_shared<HMMWV_ReissnerTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_ReissnerTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_ReissnerTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_ReissnerTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
    }

    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->SetCollisionType(m_tire_collision_type);
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
        }
    }

    m_vehicle->EnableBrakeLocking(m_brake_locking);

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
void HMMWV::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void HMMWV::Advance(double step) {
    m_vehicle->Advance(step);
}

// =============================================================================

HMMWV_Full::HMMWV_Full() : m_rigidColumn(false), m_use_tierod_bodies(false) {
    m_steeringType = SteeringTypeWV::PITMAN_ARM;
}

HMMWV_Full::HMMWV_Full(ChSystem* system) : HMMWV(system), m_rigidColumn(false), m_use_tierod_bodies(false) {
    m_steeringType = SteeringTypeWV::PITMAN_ARM;
}

HMMWV_Vehicle* HMMWV_Full::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleFull(m_system, m_fixed, m_driveType, m_brake_type, m_steeringType, m_use_tierod_bodies,
                                     m_rigidColumn, m_chassisCollisionType);
    }

    return new HMMWV_VehicleFull(m_fixed, m_driveType, m_brake_type, m_steeringType, m_use_tierod_bodies, m_rigidColumn,
                                 m_contactMethod, m_chassisCollisionType);
}

HMMWV_Reduced::HMMWV_Reduced() {
    m_steeringType = SteeringTypeWV::RACK_PINION;
}

HMMWV_Reduced::HMMWV_Reduced(ChSystem* system) : HMMWV(system) {
    m_steeringType = SteeringTypeWV::RACK_PINION;
}

HMMWV_Vehicle* HMMWV_Reduced::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleReduced(m_system, m_fixed, m_driveType, m_brake_type, m_steeringType,
                                        m_chassisCollisionType);
    }

    return new HMMWV_VehicleReduced(m_fixed, m_driveType, m_brake_type, m_steeringType, m_contactMethod,
                                    m_chassisCollisionType);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
