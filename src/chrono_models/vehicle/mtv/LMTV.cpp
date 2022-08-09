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
// Wrapper classes for modeling an entire LMTV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/LMTV.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
LMTV::LMTV()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

LMTV::LMTV(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_steeringType(SteeringTypeWV::PITMAN_ARM),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

LMTV::~LMTV() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void LMTV::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void LMTV::Initialize() {
    // Create and initialize the LMTV vehicle
    m_vehicle = m_system
                    ? new LMTV_Vehicle(m_system, m_fixed, m_brake_type, m_steeringType, m_chassisCollisionType)
                    : new LMTV_Vehicle(m_fixed, m_brake_type, m_steeringType, m_contactMethod, m_chassisCollisionType);

    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case PowertrainModelType::SIMPLE_MAP: {
            auto powertrain = chrono_types::make_shared<FMTV_SimpleMapPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_CVT: {
            auto powertrain = chrono_types::make_shared<FMTV_SimpleCVTPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE: {
            auto powertrain = chrono_types::make_shared<FMTV_SimplePowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SHAFTS: {
            auto powertrain = chrono_types::make_shared<FMTV_Powertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<FMTV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<FMTV_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<FMTV_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<FMTV_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<FMTV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<FMTV_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<FMTV_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<FMTV_TMeasyTire>("RR");

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

    m_vehicle->EnableBrakeLocking(m_brake_locking);

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
void LMTV::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

void LMTV::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
