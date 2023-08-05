// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Wrapper classes for modeling an entire Duro vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/duro/Duro.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
Duro::Duro()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::SIMPLE_MAP),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

Duro::Duro(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::SIMPLE_MAP),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

Duro::~Duro() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void Duro::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Duro::Initialize() {
    // Create and initialize the Duro vehicle
    m_vehicle = m_system
                    ? new Duro_Vehicle(m_system, m_fixed, m_brake_type, m_steeringType, m_chassisCollisionType)
                    : new Duro_Vehicle(m_fixed, m_brake_type, m_steeringType, m_contactMethod, m_chassisCollisionType);

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
            engine = chrono_types::make_shared<Duro_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            // engine = chrono_types::make_shared<Duro_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            // engine = chrono_types::make_shared<Duro_EngineSimple>("Engine");
            break;
    }

    switch (m_transmissionType) {
        case TransmissionModelType::SHAFTS:
            transmission = chrono_types::make_shared<Duro_AutomaticTransmissionShafts>("Transmission");
            break;
        case TransmissionModelType::SIMPLE_MAP:
            // transmission = chrono_types::make_shared<Duro_AutomaticTransmissionSimpleMap>("Transmission");
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

            auto tire_FL = chrono_types::make_shared<Duro_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<Duro_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<Duro_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<Duro_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<Duro_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<Duro_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<Duro_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<Duro_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        default:
            GetLog() << "Unsupported Tire Model Type! Switching to TMsimple.\n";
        case TireModelType::TMSIMPLE: {
            auto tire_FL = chrono_types::make_shared<Duro_TMsimpleTire>("FL");
            auto tire_FR = chrono_types::make_shared<Duro_TMsimpleTire>("FR");
            auto tire_RL = chrono_types::make_shared<Duro_TMsimpleTire>("RL");
            auto tire_RR = chrono_types::make_shared<Duro_TMsimpleTire>("RR");

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

    m_vehicle->EnableBrakeLocking(m_brake_locking);

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
void Duro::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}
// -----------------------------------------------------------------------------
void Duro::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
