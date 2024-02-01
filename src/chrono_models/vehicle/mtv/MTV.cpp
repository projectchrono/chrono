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
// Wrapper classes for modeling an entire MTV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/MTV.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
MTV::MTV()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_use_walking_beam(false),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0, 0, 0}),
      m_apply_drag(false) {}

MTV::MTV(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_engineType(EngineModelType::SHAFTS),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_use_walking_beam(false),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0, 0, 0}),
      m_apply_drag(false) {}

MTV::~MTV() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void MTV::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void MTV::Initialize() {
    // Create and initialize the MTV vehicle
    m_vehicle =
        m_system ? new MTV_Vehicle(m_system, m_fixed, m_use_walking_beam, m_brake_type, m_chassisCollisionType)
                 : new MTV_Vehicle(m_fixed, m_use_walking_beam, m_brake_type, m_contactMethod, m_chassisCollisionType);

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
            engine = chrono_types::make_shared<FMTV_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<FMTV_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<FMTV_EngineSimple>("Engine");
            transmission = chrono_types::make_shared<FMTV_AutomaticTransmissionSimple>("Transmission");
            break;
    }
    if (!transmission) {
        switch (m_transmissionType) {
            case TransmissionModelType::AUTOMATIC_SHAFTS:
                transmission = chrono_types::make_shared<FMTV_AutomaticTransmissionShafts>("Transmission");
                break;
            case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
                transmission = chrono_types::make_shared<FMTV_AutomaticTransmissionSimpleMap>("Transmission");
                break;
        }
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

            auto tire_FL = chrono_types::make_shared<FMTV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<FMTV_RigidTire>("FR", use_mesh);
            auto tire_RL1 = chrono_types::make_shared<FMTV_RigidTire>("RL1", use_mesh);
            auto tire_RR1 = chrono_types::make_shared<FMTV_RigidTire>("RR1", use_mesh);
            auto tire_RL2 = chrono_types::make_shared<FMTV_RigidTire>("RL2", use_mesh);
            auto tire_RR2 = chrono_types::make_shared<FMTV_RigidTire>("RR2", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(2)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }

        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<FMTV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<FMTV_TMeasyTire>("FR");
            auto tire_RL1 = chrono_types::make_shared<FMTV_TMeasyTire>("RL1");
            auto tire_RR1 = chrono_types::make_shared<FMTV_TMeasyTire>("RR1");
            auto tire_RL2 = chrono_types::make_shared<FMTV_TMeasyTire>("RL2");
            auto tire_RR2 = chrono_types::make_shared<FMTV_TMeasyTire>("RR2");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(2)->m_wheels[RIGHT], VisualizationType::NONE);

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
void MTV::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}
// -----------------------------------------------------------------------------
void MTV::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
