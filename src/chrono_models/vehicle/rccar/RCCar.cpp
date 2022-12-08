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
// Wrapper classes for modeling an entire RCCar vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/rccar/RCCar.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
RCCar::RCCar()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1),
      m_voltage_ratio(1),
      m_rolling_friction_coeff(0.05) {}

RCCar::RCCar(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1),
      m_voltage_ratio(1) {}

RCCar::~RCCar() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void RCCar::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void RCCar::Initialize() {
    // Create and initialize the RCCar vehicle
    m_vehicle = m_system ? new RCCar_Vehicle(m_system, m_fixed, m_chassisCollisionType)
                         : new RCCar_Vehicle(m_fixed, m_contactMethod, m_chassisCollisionType);

    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<RCCar_SimpleMapPowertrain>("Powertrain");
    powertrain->m_voltage_ratio = m_voltage_ratio;
    powertrain->m_stall_torque = m_stall_torque;
    powertrain->m_motor_resistance_c0 = m_motor_resistance_c0;
    powertrain->m_motor_resistance_c1 = m_motor_resistance_c1;
    m_vehicle->InitializePowertrain(powertrain);

    // Create the tires and set parameters depending on type.
    bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
    switch (m_tireType) {
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<RCCar_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<RCCar_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<RCCar_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<RCCar_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            tire_FL->SetRollingResistanceCoefficients(m_rolling_friction_coeff, m_rolling_friction_coeff);
            tire_FR->SetRollingResistanceCoefficients(m_rolling_friction_coeff, m_rolling_friction_coeff);
            tire_RL->SetRollingResistanceCoefficients(m_rolling_friction_coeff, m_rolling_friction_coeff);
            tire_RR->SetRollingResistanceCoefficients(m_rolling_friction_coeff, m_rolling_friction_coeff);

            m_tire_mass = tire_FL->GetMass();
            break;
        }
        default: {  // Defaults to rigid tires
            auto tire_FL = chrono_types::make_shared<RCCar_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<RCCar_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<RCCar_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<RCCar_RigidTire>("RR", use_mesh);

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
void RCCar::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void RCCar::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono
