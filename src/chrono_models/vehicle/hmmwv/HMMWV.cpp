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
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_PacejkaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Powertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
HMMWV::HMMWV()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_driveType(DrivelineType::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::HMMWV(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_brake_locking(false),
      m_brake_type(BrakeType::SIMPLE),
      m_driveType(DrivelineType::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::~HMMWV() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void HMMWV::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void HMMWV::Initialize() {
    // Create and initialize the HMMWV vehicle
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
            auto powertrain = chrono_types::make_shared<HMMWV_Powertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_MAP: {
            auto powertrain = chrono_types::make_shared<HMMWV_SimpleMapPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE: {
            auto powertrain = chrono_types::make_shared<HMMWV_SimplePowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
        case PowertrainModelType::SIMPLE_CVT: {
            auto powertrain = chrono_types::make_shared<HMMWV_SimpleCVTPowertrain>("Powertrain");
            m_vehicle->InitializePowertrain(powertrain);
            break;
        }
    }

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);

            auto tire_FL = chrono_types::make_shared<HMMWV_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<HMMWV_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<HMMWV_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<HMMWV_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::LUGRE: {
            auto tire_FL = chrono_types::make_shared<HMMWV_LugreTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_LugreTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_LugreTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_LugreTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::FIALA: {
            auto tire_FL = chrono_types::make_shared<HMMWV_FialaTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_FialaTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_FialaTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_FialaTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<HMMWV_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::PAC89: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac89Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac89Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac89Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac89Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::PAC02: {
            auto tire_FL = chrono_types::make_shared<HMMWV_Pac02Tire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_Pac02Tire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_Pac02Tire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_Pac02Tire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::PACEJKA: {
            auto tire_FL = chrono_types::make_shared<HMMWV_PacejkaTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_PacejkaTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_PacejkaTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_PacejkaTire>("RR");

            tire_FL->SetDrivenWheel(false);
            tire_FR->SetDrivenWheel(false);
            tire_RL->SetDrivenWheel(true);
            tire_RR->SetDrivenWheel(true);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::ANCF: {
            auto tire_FL = chrono_types::make_shared<HMMWV_ANCFTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_ANCFTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_ANCFTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_ANCFTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

            break;
        }
        case TireModelType::REISSNER: {
            auto tire_FL = chrono_types::make_shared<HMMWV_ReissnerTire>("FL");
            auto tire_FR = chrono_types::make_shared<HMMWV_ReissnerTire>("FR");
            auto tire_RL = chrono_types::make_shared<HMMWV_ReissnerTire>("RL");
            auto tire_RR = chrono_types::make_shared<HMMWV_ReissnerTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);

            m_tire_mass = tire_FL->ReportMass();

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
}

// -----------------------------------------------------------------------------
void HMMWV::SetTireVisualizationType(VisualizationType vis) {
    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->SetVisualizationType(vis);
        }
    }
}

// -----------------------------------------------------------------------------
void HMMWV::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void HMMWV::Advance(double step) {
    m_vehicle->Advance(step);
}

// -----------------------------------------------------------------------------
double HMMWV::GetTotalMass() const {
    return m_vehicle->GetVehicleMass() + 4 * m_tire_mass;
}

// =============================================================================

HMMWV_Vehicle* HMMWV_Full::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleFull(m_system, m_fixed, m_driveType, m_brake_type, m_steeringType, m_rigidColumn,
                                     m_chassisCollisionType);
    }

    return new HMMWV_VehicleFull(m_fixed, m_driveType, m_brake_type, m_steeringType, m_rigidColumn, m_contactMethod,
                                 m_chassisCollisionType);
}

HMMWV_Vehicle* HMMWV_Reduced::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleReduced(m_system, m_fixed, m_driveType, m_brake_type, m_chassisCollisionType);
    }

    return new HMMWV_VehicleReduced(m_fixed, m_driveType, m_brake_type, m_contactMethod, m_chassisCollisionType);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
