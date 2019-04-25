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
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
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
    : m_system(NULL),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({{NULL, NULL, NULL, NULL}}),
      m_contactMethod(ChMaterialSurface::NSC),
      m_chassisCollisionType(ChassisCollisionType::NONE),
      m_fixed(false),
      m_driveType(DrivelineType::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_vehicle_step_size(-1),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::HMMWV(ChSystem* system)
    : m_system(system),
      m_vehicle(NULL),
      m_powertrain(NULL),
      m_tires({{NULL, NULL, NULL, NULL}}),
      m_contactMethod(ChMaterialSurface::NSC),
      m_chassisCollisionType(ChassisCollisionType::NONE),
      m_fixed(false),
      m_driveType(DrivelineType::AWD),
      m_powertrainType(PowertrainModelType::SHAFTS),
      m_tireType(TireModelType::RIGID),
      m_tire_collision_type(ChTire::CollisionType::SINGLE_POINT),
      m_vehicle_step_size(-1),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false) {}

HMMWV::~HMMWV() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_tires[0];
    delete m_tires[1];
    delete m_tires[2];
    delete m_tires[3];
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

    if (m_vehicle_step_size > 0) {
        m_vehicle->SetStepsize(m_vehicle_step_size);
    }

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case PowertrainModelType::SHAFTS: {
            HMMWV_Powertrain* ptrain = new HMMWV_Powertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
        case PowertrainModelType::SIMPLE_MAP: {
            HMMWV_SimpleMapPowertrain* ptrain = new HMMWV_SimpleMapPowertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
        case PowertrainModelType::SIMPLE: {
            HMMWV_SimplePowertrain* ptrain = new HMMWV_SimplePowertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
        case PowertrainModelType::SIMPLE_CVT: {
            HMMWV_SimpleCVTPowertrain* ptrain = new HMMWV_SimpleCVTPowertrain("Powertrain");
            m_powertrain = ptrain;
            break;
        }
    }

    m_powertrain->Initialize(GetChassisBody(), m_vehicle->GetDriveshaft());

    // Create the tires and set parameters depending on type.
    switch (m_tireType) {
        case TireModelType::RIGID:
        case TireModelType::RIGID_MESH: {
            bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
            HMMWV_RigidTire* tire_FL = new HMMWV_RigidTire("FL", use_mesh);
            HMMWV_RigidTire* tire_FR = new HMMWV_RigidTire("FR", use_mesh);
            HMMWV_RigidTire* tire_RL = new HMMWV_RigidTire("RL", use_mesh);
            HMMWV_RigidTire* tire_RR = new HMMWV_RigidTire("RR", use_mesh);

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::LUGRE: {
            HMMWV_LugreTire* tire_FL = new HMMWV_LugreTire("FL");
            HMMWV_LugreTire* tire_FR = new HMMWV_LugreTire("FR");
            HMMWV_LugreTire* tire_RL = new HMMWV_LugreTire("RL");
            HMMWV_LugreTire* tire_RR = new HMMWV_LugreTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::FIALA: {
            HMMWV_FialaTire* tire_FL = new HMMWV_FialaTire("FL");
            HMMWV_FialaTire* tire_FR = new HMMWV_FialaTire("FR");
            HMMWV_FialaTire* tire_RL = new HMMWV_FialaTire("RL");
            HMMWV_FialaTire* tire_RR = new HMMWV_FialaTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::TMEASY: {
            HMMWV_TMeasyTire* tire_FL = new HMMWV_TMeasyTire("FL");
            HMMWV_TMeasyTire* tire_FR = new HMMWV_TMeasyTire("FR");
            HMMWV_TMeasyTire* tire_RL = new HMMWV_TMeasyTire("RL");
            HMMWV_TMeasyTire* tire_RR = new HMMWV_TMeasyTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::PAC89: {
            HMMWV_Pac89Tire* tire_FL = new HMMWV_Pac89Tire("FL");
            HMMWV_Pac89Tire* tire_FR = new HMMWV_Pac89Tire("FR");
            HMMWV_Pac89Tire* tire_RL = new HMMWV_Pac89Tire("RL");
            HMMWV_Pac89Tire* tire_RR = new HMMWV_Pac89Tire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::PACEJKA: {
            ChPacejkaTire* tire_FL = new HMMWV_Pac02Tire("FL");
            ChPacejkaTire* tire_FR = new HMMWV_Pac02Tire("FR");
            ChPacejkaTire* tire_RL = new HMMWV_Pac02Tire("RL");
            ChPacejkaTire* tire_RR = new HMMWV_Pac02Tire("RR");

            tire_FL->SetDrivenWheel(false);
            tire_FR->SetDrivenWheel(false);
            tire_RL->SetDrivenWheel(true);
            tire_RR->SetDrivenWheel(true);

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;

            break;
        }
        case TireModelType::ANCF: {
            HMMWV_ANCFTire* tire_FL = new HMMWV_ANCFTire("FL");
            HMMWV_ANCFTire* tire_FR = new HMMWV_ANCFTire("FR");
            HMMWV_ANCFTire* tire_RL = new HMMWV_ANCFTire("RL");
            HMMWV_ANCFTire* tire_RR = new HMMWV_ANCFTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
            break;
        }
        case TireModelType::REISSNER: {
            HMMWV_ReissnerTire* tire_FL = new HMMWV_ReissnerTire("FL");
            HMMWV_ReissnerTire* tire_FR = new HMMWV_ReissnerTire("FR");
            HMMWV_ReissnerTire* tire_RL = new HMMWV_ReissnerTire("RL");
            HMMWV_ReissnerTire* tire_RR = new HMMWV_ReissnerTire("RR");

            m_tires[0] = tire_FL;
            m_tires[1] = tire_FR;
            m_tires[2] = tire_RL;
            m_tires[3] = tire_RR;
            break;
        }
        default:
            break;
    }

    // Initialize the tires.
    m_tires[0]->Initialize(m_vehicle->GetWheelBody(FRONT_LEFT), LEFT);
    m_tires[1]->Initialize(m_vehicle->GetWheelBody(FRONT_RIGHT), RIGHT);
    m_tires[2]->Initialize(m_vehicle->GetWheelBody(REAR_LEFT), LEFT);
    m_tires[3]->Initialize(m_vehicle->GetWheelBody(REAR_RIGHT), RIGHT);

    if (m_tire_step_size > 0) {
        m_tires[0]->SetStepsize(m_tire_step_size);
        m_tires[1]->SetStepsize(m_tire_step_size);
        m_tires[2]->SetStepsize(m_tire_step_size);
        m_tires[3]->SetStepsize(m_tire_step_size);
    }

    m_tire_mass = m_tires[0]->ReportMass();
}

// -----------------------------------------------------------------------------
void HMMWV::SetTireVisualizationType(VisualizationType vis) {
    m_tires[0]->SetVisualizationType(vis);
    m_tires[1]->SetVisualizationType(vis);
    m_tires[2]->SetVisualizationType(vis);
    m_tires[3]->SetVisualizationType(vis);
}

// -----------------------------------------------------------------------------
void HMMWV::Synchronize(double time,
                        double steering_input,
                        double braking_input,
                        double throttle_input,
                        const ChTerrain& terrain) {
    TerrainForces tire_forces(4);
    WheelState wheel_states[4];

    tire_forces[0] = m_tires[0]->GetTireForce();
    tire_forces[1] = m_tires[1]->GetTireForce();
    tire_forces[2] = m_tires[2]->GetTireForce();
    tire_forces[3] = m_tires[3]->GetTireForce();

    wheel_states[0] = m_vehicle->GetWheelState(FRONT_LEFT);
    wheel_states[1] = m_vehicle->GetWheelState(FRONT_RIGHT);
    wheel_states[2] = m_vehicle->GetWheelState(REAR_LEFT);
    wheel_states[3] = m_vehicle->GetWheelState(REAR_RIGHT);

    double powertrain_torque = m_powertrain->GetOutputTorque();

    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();

    m_tires[0]->Synchronize(time, wheel_states[0], terrain, m_tire_collision_type);
    m_tires[1]->Synchronize(time, wheel_states[1], terrain, m_tire_collision_type);
    m_tires[2]->Synchronize(time, wheel_states[2], terrain, m_tire_collision_type);
    m_tires[3]->Synchronize(time, wheel_states[3], terrain, m_tire_collision_type);

    m_powertrain->Synchronize(time, throttle_input, driveshaft_speed);

    m_vehicle->Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
}

// -----------------------------------------------------------------------------
void HMMWV::Advance(double step) {
    m_tires[0]->Advance(step);
    m_tires[1]->Advance(step);
    m_tires[2]->Advance(step);
    m_tires[3]->Advance(step);

    m_powertrain->Advance(step);

    m_vehicle->Advance(step);
}

// -----------------------------------------------------------------------------
double HMMWV::GetTotalMass() const {
    return m_vehicle->GetVehicleMass() + 4 * m_tire_mass;
}

// =============================================================================

HMMWV_Vehicle* HMMWV_Full::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleFull(m_system, m_fixed, m_driveType, m_steeringType, m_rigidColumn,
                                     m_chassisCollisionType);
    }

    return new HMMWV_VehicleFull(m_fixed, m_driveType, m_steeringType, m_rigidColumn, m_contactMethod,
                                 m_chassisCollisionType);
}

HMMWV_Vehicle* HMMWV_Reduced::CreateVehicle() {
    if (m_system) {
        return new HMMWV_VehicleReduced(m_system, m_fixed, m_driveType, m_chassisCollisionType);
    }

    return new HMMWV_VehicleReduced(m_fixed, m_driveType, m_contactMethod, m_chassisCollisionType);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
