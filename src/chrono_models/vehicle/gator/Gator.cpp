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
// Authors: Radu Serban
// =============================================================================
//
// Wrapper classes for modeling an entire Gator vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimple.h"
#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/gator/Gator_EngineSimple.h"
#include "chrono_models/vehicle/gator/Gator_EngineSimpleMap.h"
#include "chrono_models/vehicle/gator/Gator_RigidTire.h"
#include "chrono_models/vehicle/gator/Gator_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
Gator::Gator()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contact_method(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassis_collision_type(CollisionType::NONE),
      m_fixed(false),
      m_driveline_type(DrivelineTypeWV::SIMPLE),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tire_type(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_tire_mass(0),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_Cd(0),
      m_area(0),
      m_air_density(0) {}

Gator::Gator(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contact_method(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassis_collision_type(CollisionType::NONE),
      m_fixed(false),
      m_driveline_type(DrivelineTypeWV::SIMPLE),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tire_type(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_tire_mass(0),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_Cd(0),
      m_area(0),
      m_air_density(0) {}

Gator::~Gator() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void Gator::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Gator::Initialize() {
    // Create and initialize the Gator vehicle
    if (m_system) {
        m_vehicle = new Gator_Vehicle(m_system, m_fixed, m_driveline_type, m_brake_type, m_chassis_collision_type);
    } else {
        m_vehicle =
            new Gator_Vehicle(m_fixed, m_driveline_type, m_brake_type, m_contact_method, m_chassis_collision_type);
    }
    m_vehicle->SetCollisionSystemType(m_collsysType);
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    if (true) {  // set this to true/false to select one of the two powertrains below
        auto engine = chrono_types::make_shared<Gator_EngineSimple>("Engine");
        auto transmission = chrono_types::make_shared<Gator_AutomaticTransmissionSimple>("Transmission");
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    } else {
        auto engine = chrono_types::make_shared<Gator_EngineSimpleMap>("Engine");
        auto transmission = chrono_types::make_shared<Gator_AutomaticTransmissionSimpleMap>("Transmission");
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Create the tires and set parameters depending on type.
    switch (m_tire_type) {
        case TireModelType::RIGID_MESH:
        case TireModelType::RIGID: {
            bool use_mesh = (m_tire_type == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<Gator_RigidTire_Front>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<Gator_RigidTire_Front>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<Gator_RigidTire_Rear>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<Gator_RigidTire_Rear>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = 2 * (tire_FL->GetMass() + tire_RL->GetMass());

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<Gator_TMeasyTire_Front>("FL");
            auto tire_FR = chrono_types::make_shared<Gator_TMeasyTire_Front>("FR");
            auto tire_RL = chrono_types::make_shared<Gator_TMeasyTire_Rear>("RL");
            auto tire_RR = chrono_types::make_shared<Gator_TMeasyTire_Rear>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = 2 * (tire_FL->GetMass() + tire_RL->GetMass());

            break;
        }

        default:
            break;
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
void Gator::SetWheelVisualizationType(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_vehicle->SetWheelVisualizationType(VisualizationType::NONE);
        return;
    }
    m_vehicle->SetWheelVisualizationType(vis);
}

// -----------------------------------------------------------------------------
void Gator::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void Gator::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
