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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// Wrapper classes for modeling an entire CityBus vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/citybus/CityBus.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
CityBus::CityBus()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::TMEASY),
      m_tire_step_size(-1),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

CityBus::CityBus(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

CityBus::~CityBus() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void CityBus::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void CityBus::Initialize() {
    // Create and initialize the CityBus vehicle
    m_vehicle = m_system ? new CityBus_Vehicle(m_system, m_fixed, m_brake_type, m_chassisCollisionType)
                         : new CityBus_Vehicle(m_fixed, m_brake_type, m_contactMethod, m_chassisCollisionType);

    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<CityBus_SimpleMapPowertrain>("Powertrain");
    m_vehicle->InitializePowertrain(powertrain);

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID_MESH:
        case TireModelType::RIGID: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<CityBus_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<CityBus_RigidTire>("FR", use_mesh);

            auto tire_RLi = chrono_types::make_shared<CityBus_RigidTire>("RLi", use_mesh);
            auto tire_RRi = chrono_types::make_shared<CityBus_RigidTire>("RRi", use_mesh);

            auto tire_RLo = chrono_types::make_shared<CityBus_RigidTire>("RLo", use_mesh);
            auto tire_RRo = chrono_types::make_shared<CityBus_RigidTire>("RRo", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLi, m_vehicle->GetAxle(1)->GetWheel(LEFT, INNER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRi, m_vehicle->GetAxle(1)->GetWheel(RIGHT, INNER), VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLo, m_vehicle->GetAxle(1)->GetWheel(LEFT, OUTER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRo, m_vehicle->GetAxle(1)->GetWheel(RIGHT, OUTER), VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<CityBus_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<CityBus_TMeasyTire>("FR");

            auto tire_RLi = chrono_types::make_shared<CityBus_TMeasyTire>("RLi");
            auto tire_RRi = chrono_types::make_shared<CityBus_TMeasyTire>("RRi");

            auto tire_RLo = chrono_types::make_shared<CityBus_TMeasyTire>("RLo");
            auto tire_RRo = chrono_types::make_shared<CityBus_TMeasyTire>("RRo");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLi, m_vehicle->GetAxle(1)->GetWheel(LEFT, INNER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRi, m_vehicle->GetAxle(1)->GetWheel(RIGHT, INNER), VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLo, m_vehicle->GetAxle(1)->GetWheel(LEFT, OUTER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRo, m_vehicle->GetAxle(1)->GetWheel(RIGHT, OUTER), VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<CityBus_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<CityBus_Pac02Tire>("FR");

            auto tire_RLi = chrono_types::make_shared<CityBus_Pac02Tire>("RLi");
            auto tire_RRi = chrono_types::make_shared<CityBus_Pac02Tire>("RRi");

            auto tire_RLo = chrono_types::make_shared<CityBus_Pac02Tire>("RLo");
            auto tire_RRo = chrono_types::make_shared<CityBus_Pac02Tire>("RRo");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLi, m_vehicle->GetAxle(1)->GetWheel(LEFT, INNER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRi, m_vehicle->GetAxle(1)->GetWheel(RIGHT, INNER), VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLo, m_vehicle->GetAxle(1)->GetWheel(LEFT, OUTER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRo, m_vehicle->GetAxle(1)->GetWheel(RIGHT, OUTER), VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        default:
            break;
    }

    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
        }
    }

    m_vehicle->EnableBrakeLocking(m_brake_locking);
}

// -----------------------------------------------------------------------------
void CityBus::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void CityBus::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
