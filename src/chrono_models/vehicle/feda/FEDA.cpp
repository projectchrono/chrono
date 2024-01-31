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
// Wrapper classes for modeling an entire Sedan vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/feda/FEDA.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
FEDA::FEDA()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_ride_height_config(1),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_pressure(60 * 6894.76),
      m_damper_mode(1) {}

FEDA::FEDA(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_ride_height_config(1),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_pressure(60 * 6894.76),
      m_damper_mode(1) {}

FEDA::~FEDA() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void FEDA::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

void FEDA::SetDamperMode(DamperMode theDamperMode) {
    switch (theDamperMode) {
        case DamperMode::FSD:
            m_damper_mode = 1;
            break;
        case DamperMode::PASSIVE_LOW:
            m_damper_mode = 2;
            break;
        case DamperMode::PASSIVE_HIGH:
            m_damper_mode = 3;
            break;
    }
}

// -----------------------------------------------------------------------------
void FEDA::Initialize() {
    // Create and initialize the Sedan vehicle
    GetLog() << "FEDA::Initialize(): Damper Mode = " << m_damper_mode << "\n";
    m_vehicle = m_system ? new FEDA_Vehicle(m_system, m_fixed, m_brake_type, m_chassisCollisionType,
                                            m_ride_height_config, m_damper_mode)
                         : new FEDA_Vehicle(m_fixed, m_brake_type, m_contactMethod, m_chassisCollisionType,
                                            m_ride_height_config, m_damper_mode);

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
            // engine = chrono_types::make_shared<FEDA_EngineShafts>("Engine");
            GetLog() << "EngineModelType::SHAFTS not implemented for this model.\n";
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<FEDA_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            // engine = chrono_types::make_shared<FEDA_EngineSimple>("Engine");
            GetLog() << "EngineModelType::SIMPLE not implemented for this model.\n";
            break;
    }

    switch (m_transmissionType) {
        case TransmissionModelType::AUTOMATIC_SHAFTS:
            // transmission = chrono_types::make_shared<FEDA_AutomaticTransmissionShafts>("Transmission");
            GetLog() << "TransmissionModelType::AUTOMATIC_SHAFTS not implemented for this model.\n";
            break;
        case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
            transmission = chrono_types::make_shared<FEDA_AutomaticTransmissionSimpleMap>("Transmission");
            break;
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID_MESH:
        case TireModelType::RIGID: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<FEDA_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<FEDA_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<FEDA_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<FEDA_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<FEDA_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<FEDA_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<FEDA_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<FEDA_Pac02Tire>("RR");

            tire_FL->SetPressure(m_tire_pressure);
            tire_FR->SetPressure(m_tire_pressure);
            tire_RL->SetPressure(m_tire_pressure);
            tire_RR->SetPressure(m_tire_pressure);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE,
                                      m_tire_collision_type);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        default:
            GetLog() << "Unknown Tire Model Type! Switching to TMsimple.\n";
        case TireModelType::TMSIMPLE: {
            auto tire_FL = chrono_types::make_shared<FEDA_TMsimpleTire>("FL");
            auto tire_FR = chrono_types::make_shared<FEDA_TMsimpleTire>("FR");
            auto tire_RL = chrono_types::make_shared<FEDA_TMsimpleTire>("RL");
            auto tire_RR = chrono_types::make_shared<FEDA_TMsimpleTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE,
                                      m_tire_collision_type);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE,
                                      m_tire_collision_type);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<FEDA_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<FEDA_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<FEDA_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<FEDA_TMeasyTire>("RR");

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
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
        }
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
void FEDA::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void FEDA::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
