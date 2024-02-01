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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He, Rainer Gericke
// =============================================================================
//
// Wrapper classes for modeling an entire MAN 5t vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// The MAN Kat 1 truck family has been designed for offroad service.
// The development stems from the 60s, the begin of servce was ca. 1976
//
// The model data come from publicly available sources, fora of private Kat 1
// users and the book:
// P. Ocker: "MAN - Die Allrad-Alleskönner", Heel Verlag, 1999, ISBN 3-89365-705-3
//
// The 10t (load capacity) version has four driven rigid axles. The model is unloaded.
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/man/MAN_10t.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
MAN_10t::MAN_10t()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_use_8WD_drivetrain(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::TMEASY),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0, 0, 0, 0, 0}),
      m_apply_drag(false) {}

MAN_10t::MAN_10t(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_use_8WD_drivetrain(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_tireType(TireModelType::TMEASY),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0, 0, 0, 0, 0}),
      m_apply_drag(false) {}

MAN_10t::~MAN_10t() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void MAN_10t::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void MAN_10t::Initialize() {
    // Create and initialize the MAN_10t vehicle
    m_vehicle =
        m_system
            ? new MAN_10t_Vehicle(m_system, m_fixed, m_brake_type, m_chassisCollisionType, m_use_8WD_drivetrain)
            : new MAN_10t_Vehicle(m_fixed, m_brake_type, m_contactMethod, m_chassisCollisionType, m_use_8WD_drivetrain);

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
            // engine = chrono_types::make_shared<MAN_7t_EngineShafts>("Engine");
            GetLog() << "EngineModelType::SHAFTS not implemented for this model.\n";
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<MAN_7t_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            engine = chrono_types::make_shared<MAN_7t_EngineSimple>("Engine");
            transmission = chrono_types::make_shared<MAN_7t_AutomaticTransmissionSimple>("Transmission");
            break;
    }

    if (!transmission) {
        switch (m_transmissionType) {
            case TransmissionModelType::AUTOMATIC_SHAFTS:
                // transmission = chrono_types::make_shared<MAN_7t_AutomaticTransmissionShafts>("Transmission");
                GetLog() << "TransmissionModelType::AUTOMATIC_SHAFTS not implemented for this model.\n";
                break;
            case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
                transmission = chrono_types::make_shared<MAN_7t_AutomaticTransmissionSimpleMap>("Transmission");
                break;
        }
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_vehicle->InitializePowertrain(powertrain);
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        /*
        case TireModelType::RIGID_MESH:
        case TireModelType::RIGID: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<MAN_5t_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<MAN_5t_RigidTire>("FR", use_mesh);

            auto tire_RLi = chrono_types::make_shared<MAN_5t_RigidTire>("RLi", use_mesh);
            auto tire_RRi = chrono_types::make_shared<MAN_5t_RigidTire>("RRi", use_mesh);

            auto tire_RLo = chrono_types::make_shared<MAN_5t_RigidTire>("RLo", use_mesh);
            auto tire_RRo = chrono_types::make_shared<MAN_5t_RigidTire>("RRo", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLi, m_vehicle->GetAxle(1)->GetWheel(LEFT, INNER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRi, m_vehicle->GetAxle(1)->GetWheel(RIGHT, INNER), VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RLo, m_vehicle->GetAxle(1)->GetWheel(LEFT, OUTER), VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RRo, m_vehicle->GetAxle(1)->GetWheel(RIGHT, OUTER), VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
*/
        case TireModelType::TMEASY: {
            auto tire_FL1 = chrono_types::make_shared<MAN_5t_TMeasyTire>("FL1");
            auto tire_FR1 = chrono_types::make_shared<MAN_5t_TMeasyTire>("FR1");

            auto tire_FL2 = chrono_types::make_shared<MAN_5t_TMeasyTire>("FL2");
            auto tire_FR2 = chrono_types::make_shared<MAN_5t_TMeasyTire>("FR2");

            auto tire_RL1 = chrono_types::make_shared<MAN_5t_TMeasyTire>("RL1");
            auto tire_RR1 = chrono_types::make_shared<MAN_5t_TMeasyTire>("RR1");

            auto tire_RL2 = chrono_types::make_shared<MAN_5t_TMeasyTire>("RL2");
            auto tire_RR2 = chrono_types::make_shared<MAN_5t_TMeasyTire>("RR2");

            m_vehicle->InitializeTire(tire_FL1, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR1, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_FL2, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR2, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(2)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(3)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(3)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL1->GetMass();

            break;
        }

        case TireModelType::TMSIMPLE: {
            auto tire_FL1 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("FL1");
            auto tire_FR1 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("FR1");

            auto tire_FL2 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("FL2");
            auto tire_FR2 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("FR2");

            auto tire_RL1 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("RL1");
            auto tire_RR1 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("RR1");

            auto tire_RL2 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("RL2");
            auto tire_RR2 = chrono_types::make_shared<MAN_5t_TMsimpleTire>("RR2");

            m_vehicle->InitializeTire(tire_FL1, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR1, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_FL2, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR2, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(2)->m_wheels[RIGHT], VisualizationType::NONE);

            m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(3)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(3)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL1->GetMass();

            break;
        }

            /*
                    case TireModelType::PAC02: {
                        auto tire_FL = chrono_types::make_shared<MAN_5t_Pac02Tire>("FL");
                        auto tire_FR = chrono_types::make_shared<MAN_5t_Pac02Tire>("FR");

                        auto tire_RLi = chrono_types::make_shared<MAN_5t_Pac02Tire>("RLi");
                        auto tire_RRi = chrono_types::make_shared<MAN_5t_Pac02Tire>("RRi");

                        auto tire_RLo = chrono_types::make_shared<MAN_5t_Pac02Tire>("RLo");
                        auto tire_RRo = chrono_types::make_shared<MAN_5t_Pac02Tire>("RRo");

                        m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT],
               VisualizationType::NONE); m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT],
               VisualizationType::NONE);

                        m_vehicle->InitializeTire(tire_RLi, m_vehicle->GetAxle(1)->GetWheel(LEFT, INNER),
               VisualizationType::NONE); m_vehicle->InitializeTire(tire_RRi, m_vehicle->GetAxle(1)->GetWheel(RIGHT,
               INNER), VisualizationType::NONE);

                        m_vehicle->InitializeTire(tire_RLo, m_vehicle->GetAxle(1)->GetWheel(LEFT, OUTER),
               VisualizationType::NONE); m_vehicle->InitializeTire(tire_RRo, m_vehicle->GetAxle(1)->GetWheel(RIGHT,
               OUTER), VisualizationType::NONE);

                        m_tire_mass = tire_FL->GetMass();

                        break;
                    }
            */
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
void MAN_10t::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void MAN_10t::Advance(double step) {
    m_vehicle->Advance(step);
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
