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
// Wrapper classes for modeling an entire ARTcar vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
ARTcar::ARTcar()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1),
      m_voltage_ratio(1),
      m_rolling_friction_coeff(0.05) {}

ARTcar::ARTcar(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::TMEASY),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1),
      m_voltage_ratio(1) {}

ARTcar::~ARTcar() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void ARTcar::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void ARTcar::Initialize() {
    // Create and initialize the ARTcar vehicle
    m_vehicle = m_system ? new ARTcar_Vehicle(m_system, m_fixed, m_chassisCollisionType)
                         : new ARTcar_Vehicle(m_fixed, m_contactMethod, m_chassisCollisionType);
    m_vehicle->SetCollisionSystemType(m_collsysType);
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto engine = chrono_types::make_shared<ARTcar_EngineSimpleMap>("Engine");
    auto transmission = chrono_types::make_shared<ARTcar_AutomaticTransmissionSimpleMap>("Transmission");
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    engine->m_voltage_ratio = m_voltage_ratio;
    engine->m_stall_torque = m_stall_torque;
    m_vehicle->InitializePowertrain(powertrain);

    // Create the tires and set parameters depending on type.
    bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
    switch (m_tireType) {
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<ARTcar_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<ARTcar_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<ARTcar_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<ARTcar_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            tire_FL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_FR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);

            m_tire_mass = tire_FL->GetMass();
            break;
        }
        default: {  // Defaults to rigid tires
            auto tire_FL = chrono_types::make_shared<ARTcar_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<ARTcar_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<ARTcar_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<ARTcar_RigidTire>("RR", use_mesh);

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
void ARTcar::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void ARTcar::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono
