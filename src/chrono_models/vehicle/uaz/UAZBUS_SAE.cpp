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
// Wrapper classes for modeling an entire UAZBUS_SAE vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/uaz/UAZBUS_SAE.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
UAZBUS_SAE::UAZBUS_SAE()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

UAZBUS_SAE::UAZBUS_SAE(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initFwdVel(0),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

UAZBUS_SAE::~UAZBUS_SAE() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void UAZBUS_SAE::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void UAZBUS_SAE::Initialize() {
    // Create and initialize the UAZBUS_SAE vehicle
    m_vehicle = m_system ? new UAZBUS_SAEVehicle(m_system, m_fixed, m_steeringType, m_chassisCollisionType)
                         : new UAZBUS_SAEVehicle(m_fixed, m_steeringType, m_contactMethod, m_chassisCollisionType);

    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto powertrain = chrono_types::make_shared<UAZBUS_SimpleMapPowertrain>("powertrain");
    m_vehicle->InitializePowertrain(powertrain);

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<UAZBUS_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<UAZBUS_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<UAZBUS_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<UAZBUS_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<UAZBUS_TMeasyTireFront>("FL");
            auto tire_FR = chrono_types::make_shared<UAZBUS_TMeasyTireFront>("FR");
            auto tire_RL = chrono_types::make_shared<UAZBUS_TMeasyTireFront>("RL");
            auto tire_RR = chrono_types::make_shared<UAZBUS_TMeasyTireFront>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<UAZBUS_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<UAZBUS_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<UAZBUS_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<UAZBUS_Pac02Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

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
}

// -----------------------------------------------------------------------------
void UAZBUS_SAE::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

void UAZBUS_SAE::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
