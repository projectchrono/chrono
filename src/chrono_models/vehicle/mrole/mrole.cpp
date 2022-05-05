// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Wrapper classes for modeling an entire mrole vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mrole/mrole.h"
#include "chrono_models/vehicle/mrole/mrole_Powertrain.h"
#include "chrono_models/vehicle/mrole/mrole_RigidTire.h"
#include "chrono_models/vehicle/mrole/mrole_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/mrole/mrole_SimplePowertrain.h"
#include "chrono_models/vehicle/mrole/mrole_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/mrole/mrole_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
mrole::mrole()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_driveType(DrivelineTypeWV::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_ctis(CTIS::ROAD),
      m_apply_drag(false) {}

mrole::mrole(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_driveType(DrivelineTypeWV::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_ctis(CTIS::ROAD),
      m_apply_drag(false) {}

mrole::~mrole() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void mrole::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void mrole::Initialize() {
    // Create and initialize the mrole vehicle
    m_vehicle = CreateVehicle();
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case PowertrainModelType::SHAFTS: {
            auto powertrain = chrono_types::make_shared<mrole_Powertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_MAP: {
            auto powertrain = chrono_types::make_shared<mrole_SimpleMapPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE: {
            auto powertrain = chrono_types::make_shared<mrole_SimplePowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_CVT: {
            auto powertrain = chrono_types::make_shared<mrole_SimpleCVTPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL1 = chrono_types::make_shared<mrole_RigidTire>("FL1", use_mesh);
            auto tire_FR1 = chrono_types::make_shared<mrole_RigidTire>("FR1", use_mesh);
            auto tire_FL2 = chrono_types::make_shared<mrole_RigidTire>("FL2", use_mesh);
            auto tire_FR2 = chrono_types::make_shared<mrole_RigidTire>("FR2", use_mesh);
            auto tire_RL1 = chrono_types::make_shared<mrole_RigidTire>("RL1", use_mesh);
            auto tire_RR1 = chrono_types::make_shared<mrole_RigidTire>("RR1", use_mesh);
            auto tire_RL2 = chrono_types::make_shared<mrole_RigidTire>("RL2", use_mesh);
            auto tire_RR2 = chrono_types::make_shared<mrole_RigidTire>("RR2", use_mesh);

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
        case TireModelType::LUGRE: {
            auto tire_FL = chrono_types::make_shared<mrole_LugreTire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_LugreTire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_LugreTire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_LugreTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<mrole_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_FialaTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
         */
        case TireModelType::TMEASY: {
            switch (m_ctis) {
                case CTIS::ROAD: {
                    auto tire_FL1 = chrono_types::make_shared<mrole_TMeasyTire>("FL1");
                    auto tire_FR1 = chrono_types::make_shared<mrole_TMeasyTire>("FR1");
                    auto tire_FL2 = chrono_types::make_shared<mrole_TMeasyTire>("FL2");
                    auto tire_FR2 = chrono_types::make_shared<mrole_TMeasyTire>("FR2");
                    auto tire_RL1 = chrono_types::make_shared<mrole_TMeasyTire>("RL1");
                    auto tire_RR1 = chrono_types::make_shared<mrole_TMeasyTire>("RR1");
                    auto tire_RL2 = chrono_types::make_shared<mrole_TMeasyTire>("RL2");
                    auto tire_RR2 = chrono_types::make_shared<mrole_TMeasyTire>("RR2");

                    m_vehicle->InitializeTire(tire_FL1, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR1, m_vehicle->GetAxle(0)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FL2, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR2, m_vehicle->GetAxle(1)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(2)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(3)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(3)->m_wheels[RIGHT],
                                              VisualizationType::NONE);

                    m_tire_mass = tire_FL1->GetMass();

                } break;
                case CTIS::OFFROAD_SOIL: {
                    auto tire_FL1 = chrono_types::make_shared<mrole_TMeasyTireSoil>("FL1");
                    auto tire_FR1 = chrono_types::make_shared<mrole_TMeasyTireSoil>("FR1");
                    auto tire_FL2 = chrono_types::make_shared<mrole_TMeasyTireSoil>("FL2");
                    auto tire_FR2 = chrono_types::make_shared<mrole_TMeasyTireSoil>("FR2");
                    auto tire_RL1 = chrono_types::make_shared<mrole_TMeasyTireSoil>("RL1");
                    auto tire_RR1 = chrono_types::make_shared<mrole_TMeasyTireSoil>("RR1");
                    auto tire_RL2 = chrono_types::make_shared<mrole_TMeasyTireSoil>("RL2");
                    auto tire_RR2 = chrono_types::make_shared<mrole_TMeasyTireSoil>("RR2");

                    m_vehicle->InitializeTire(tire_FL1, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR1, m_vehicle->GetAxle(0)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FL2, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR2, m_vehicle->GetAxle(1)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(2)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(3)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(3)->m_wheels[RIGHT],
                                              VisualizationType::NONE);

                    m_tire_mass = tire_FL1->GetMass();

                } break;
                case CTIS::OFFROAD_SAND: {
                    auto tire_FL1 = chrono_types::make_shared<mrole_TMeasyTireSand>("FL1");
                    auto tire_FR1 = chrono_types::make_shared<mrole_TMeasyTireSand>("FR1");
                    auto tire_FL2 = chrono_types::make_shared<mrole_TMeasyTireSand>("FL2");
                    auto tire_FR2 = chrono_types::make_shared<mrole_TMeasyTireSand>("FR2");
                    auto tire_RL1 = chrono_types::make_shared<mrole_TMeasyTireSand>("RL1");
                    auto tire_RR1 = chrono_types::make_shared<mrole_TMeasyTireSand>("RR1");
                    auto tire_RL2 = chrono_types::make_shared<mrole_TMeasyTireSand>("RL2");
                    auto tire_RR2 = chrono_types::make_shared<mrole_TMeasyTireSand>("RR2");

                    m_vehicle->InitializeTire(tire_FL1, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR1, m_vehicle->GetAxle(0)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FL2, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_FR2, m_vehicle->GetAxle(1)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL1, m_vehicle->GetAxle(2)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR1, m_vehicle->GetAxle(2)->m_wheels[RIGHT],
                                              VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RL2, m_vehicle->GetAxle(3)->m_wheels[LEFT], VisualizationType::NONE);
                    m_vehicle->InitializeTire(tire_RR2, m_vehicle->GetAxle(3)->m_wheels[RIGHT],
                                              VisualizationType::NONE);

                    m_tire_mass = tire_FL1->GetMass();

                } break;
            }

            break;
        }
        /*
        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<mrole_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_Pac89Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<mrole_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_Pac02Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::PACEJKA: {
            auto tire_FL = chrono_types::make_shared<mrole_PacejkaTire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_PacejkaTire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_PacejkaTire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_PacejkaTire>("RR");

            tire_FL->SetDrivenWheel(false);
            tire_FR->SetDrivenWheel(false);
            tire_RL->SetDrivenWheel(true);
            tire_RR->SetDrivenWheel(true);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::ANCF: {
            auto tire_FL = chrono_types::make_shared<mrole_ANCFTire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_ANCFTire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_ANCFTire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_ANCFTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
        case TireModelType::REISSNER: {
            auto tire_FL = chrono_types::make_shared<mrole_ReissnerTire>("FL");
            auto tire_FR = chrono_types::make_shared<mrole_ReissnerTire>("FR");
            auto tire_RL = chrono_types::make_shared<mrole_ReissnerTire>("RL");
            auto tire_RR = chrono_types::make_shared<mrole_ReissnerTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->GetMass();

            break;
        }
         */
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

double mrole::GetMaxTireSpeed() {
    switch (m_ctis) {
        default:
        case CTIS::ROAD:
            return 110.0 / 3.6;
        case CTIS::OFFROAD_SOIL:
            return 70.0 / 3.6;
        case CTIS::OFFROAD_SAND:
            return 30.0 / 3.6;
    }
}

// -----------------------------------------------------------------------------
void mrole::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void mrole::Advance(double step) {
    m_vehicle->Advance(step);
}

// =============================================================================

mrole_Vehicle* mrole_Full::CreateVehicle() {
    if (m_system) {
        return new mrole_VehicleFull(m_system, m_fixed, m_driveType, m_brake_type, m_steeringType, m_rigidColumn,
                                     m_chassisCollisionType);
    }

    return new mrole_VehicleFull(m_fixed, m_driveType, m_brake_type, m_steeringType, m_rigidColumn, m_contactMethod,
                                 m_chassisCollisionType);
}

mrole_Vehicle* mrole_Reduced::CreateVehicle() {
    if (m_system) {
        return new mrole_VehicleReduced(m_system, m_fixed, m_driveType, m_brake_type, m_chassisCollisionType);
    }

    return new mrole_VehicleReduced(m_fixed, m_driveType, m_brake_type, m_contactMethod, m_chassisCollisionType);
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
