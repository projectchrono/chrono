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
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"

#include "hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
HMMWV::HMMWV()
    : m_vehicle(NULL),
      m_powertrain(NULL),
      m_tireFL(NULL),
      m_tireFR(NULL),
      m_tireRL(NULL),
      m_tireRR(NULL),
      m_contactMethod(ChMaterialSurfaceBase::DVI),
      m_fixed(false),
      m_driveType(AWD),
      m_powertrainType(SHAFTS),
      m_tireType(RIGID),
      m_tire_step_size(-1),
      m_pacejkaParamFile(""),
      m_chassisVis(PRIMITIVES),
      m_wheelVis(PRIMITIVES),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)) {
}

HMMWV::~HMMWV() {
    delete m_vehicle;
    delete m_powertrain;
    delete m_tireFL;
    delete m_tireFR;
    delete m_tireRL;
    delete m_tireRR;
}

// -----------------------------------------------------------------------------
void HMMWV::Initialize() {
    // Create and initialize the HMMWV vehicle
    m_vehicle = CreateVehicle();
    m_vehicle->Initialize(m_initPos);

    // Create and initialize the powertrain system
    switch (m_powertrainType) {
        case SHAFTS: {
            HMMWV_Powertrain* ptrain = new HMMWV_Powertrain;
            ptrain->Initialize(m_vehicle->GetChassis(), m_vehicle->GetDriveshaft());
            m_powertrain = ptrain;
            break;
        }
        case SIMPLE: {
            HMMWV_SimplePowertrain* ptrain = new HMMWV_SimplePowertrain;
            ptrain->Initialize();
            m_powertrain = ptrain;
            break;
        }
    }

    // Create and initialize the tires
    switch (m_tireType) {
        case RIGID: {
            HMMWV_RigidTire* tire_FL = new HMMWV_RigidTire("FL");
            HMMWV_RigidTire* tire_FR = new HMMWV_RigidTire("FR");
            HMMWV_RigidTire* tire_RL = new HMMWV_RigidTire("RL");
            HMMWV_RigidTire* tire_RR = new HMMWV_RigidTire("RR");

            tire_FL->Initialize(m_vehicle->GetWheelBody(FRONT_LEFT));
            tire_FR->Initialize(m_vehicle->GetWheelBody(FRONT_RIGHT));
            tire_RL->Initialize(m_vehicle->GetWheelBody(REAR_LEFT));
            tire_RR->Initialize(m_vehicle->GetWheelBody(REAR_RIGHT));

            m_tireFL = tire_FL;
            m_tireFR = tire_FR;
            m_tireRL = tire_RL;
            m_tireRR = tire_RR;

            break;
        }
        case LUGRE: {
            HMMWV_LugreTire* tire_FL = new HMMWV_LugreTire("FL");
            HMMWV_LugreTire* tire_FR = new HMMWV_LugreTire("FR");
            HMMWV_LugreTire* tire_RL = new HMMWV_LugreTire("RL");
            HMMWV_LugreTire* tire_RR = new HMMWV_LugreTire("RR");

            if (m_wheelVis == NONE) {
                tire_FL->Initialize(m_vehicle->GetWheelBody(FRONT_LEFT));
                tire_FR->Initialize(m_vehicle->GetWheelBody(FRONT_RIGHT));
                tire_RL->Initialize(m_vehicle->GetWheelBody(REAR_LEFT));
                tire_RR->Initialize(m_vehicle->GetWheelBody(REAR_RIGHT));
            } else {
                tire_FL->Initialize();
                tire_FR->Initialize();
                tire_RL->Initialize();
                tire_RR->Initialize();
            }

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tireFL = tire_FL;
            m_tireFR = tire_FR;
            m_tireRL = tire_RL;
            m_tireRR = tire_RR;

            break;
        }
        case FIALA: {
            HMMWV_FialaTire* tire_FL = new HMMWV_FialaTire("FL");
            HMMWV_FialaTire* tire_FR = new HMMWV_FialaTire("FR");
            HMMWV_FialaTire* tire_RL = new HMMWV_FialaTire("RL");
            HMMWV_FialaTire* tire_RR = new HMMWV_FialaTire("RR");

            tire_FL->Initialize();
            tire_FR->Initialize();
            tire_RL->Initialize();
            tire_RR->Initialize();

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tireFL = tire_FL;
            m_tireFR = tire_FR;
            m_tireRL = tire_RL;
            m_tireRR = tire_RR;

            break;
        }
        case PACEJKA: {
            if (m_pacejkaParamFile.empty())
                throw ChException("Pacejka parameter file not specified.");

            std::string param_file = vehicle::GetDataFile(m_pacejkaParamFile);

            ChPacejkaTire* tire_FL = new ChPacejkaTire("FL", param_file);
            ChPacejkaTire* tire_FR = new ChPacejkaTire("FR", param_file);
            ChPacejkaTire* tire_RL = new ChPacejkaTire("RL", param_file);
            ChPacejkaTire* tire_RR = new ChPacejkaTire("RR", param_file);

            tire_FL->Initialize(LEFT, false);
            tire_FR->Initialize(RIGHT, false);
            tire_RL->Initialize(LEFT, true);
            tire_RR->Initialize(RIGHT, true);

            if (m_tire_step_size > 0) {
                tire_FL->SetStepsize(m_tire_step_size);
                tire_FR->SetStepsize(m_tire_step_size);
                tire_RL->SetStepsize(m_tire_step_size);
                tire_RR->SetStepsize(m_tire_step_size);
            }

            m_tireFL = tire_FL;
            m_tireFR = tire_FR;
            m_tireRL = tire_RL;
            m_tireRR = tire_RR;

            break;
        }
    }
}

// -----------------------------------------------------------------------------
void HMMWV::Synchronize(double time,
                        double steering_input,
                        double braking_input,
                        double throttle_input,
                        const ChTerrain& terrain) {
    TireForces tire_forces(4);
    WheelState wheel_states[4];

    tire_forces[0] = m_tireFL->GetTireForce();
    tire_forces[1] = m_tireFR->GetTireForce();
    tire_forces[2] = m_tireRL->GetTireForce();
    tire_forces[3] = m_tireRR->GetTireForce();

    wheel_states[0] = m_vehicle->GetWheelState(FRONT_LEFT);
    wheel_states[1] = m_vehicle->GetWheelState(FRONT_RIGHT);
    wheel_states[2] = m_vehicle->GetWheelState(REAR_LEFT);
    wheel_states[3] = m_vehicle->GetWheelState(REAR_RIGHT);

    double powertrain_torque = m_powertrain->GetOutputTorque();

    double driveshaft_speed = m_vehicle->GetDriveshaftSpeed();

    m_tireFL->Synchronize(time, wheel_states[0], terrain);
    m_tireFR->Synchronize(time, wheel_states[1], terrain);
    m_tireRL->Synchronize(time, wheel_states[2], terrain);
    m_tireRR->Synchronize(time, wheel_states[3], terrain);

    m_powertrain->Synchronize(time, throttle_input, driveshaft_speed);

    m_vehicle->Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
}

// -----------------------------------------------------------------------------
void HMMWV::Advance(double step) {
    m_tireFL->Advance(step);
    m_tireFR->Advance(step);
    m_tireRL->Advance(step);
    m_tireRR->Advance(step);

    m_powertrain->Advance(step);

    m_vehicle->Advance(step);
}

}  // end namespace hmmwv
